// tools.cc — the capabilities exposed by bf-mcp-stdio, as self-describing Tools.
//
// Each Tool binds its MCP name, description, JSON-Schema input descriptor, and
// handler in one place. Handlers are pure with respect to server state: they
// receive the request arguments and a read-only NavDatabase, and return
// {json_text, is_error}. Adding a tool means adding one entry to MakeTools();
// nothing else in the server needs to change.

#include "tools.h"

#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "core/query/query_json.h"
#include "core/routing/route.h"
#include "core/routing/route_json.h"
#include "core/routing/route_request.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace bf::mcp {

namespace {

// Parse a JSON-Schema string into a Document. The Document owns its allocator,
// so it can be moved into a Tool and kept alive there (no statics, no globals).
rapidjson::Document ParseSchema(const char* json) {
  rapidjson::Document doc;
  doc.Parse(json);
  return doc;
}

}  // namespace

// Build a tool-error JSON payload `{"error":"<message>"}` using RapidJSON's
// Writer so the message is auto-escaped. Tool error messages may carry
// user-controlled strings (an unknown departure airport, a bad route token, an
// unknown tool name), and the previous hand-rolled `R"({"error":")" + msg +
// R"("})"` concatenation let a quote/backslash in the message break the JSON
// frame.
std::string JsonError(const std::string& message) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("error");
  writer.String(message.c_str(), static_cast<unsigned>(message.size()));
  writer.EndObject();
  return buffer.GetString();
}

Tool::Tool(std::string tool_name, std::string tool_description, rapidjson::Document schema,
           ToolHandler tool_handler)
    : name(std::move(tool_name)),
      description(std::move(tool_description)),
      schema_store(std::move(schema)),
      handler(std::move(tool_handler)) {
  // input_schema borrows schema_store's allocator; copy the parsed root into it
  // so the Value and its allocator share this Tool's lifetime.
  input_schema.CopyFrom(schema_store, schema_store.GetAllocator());
}

// Serialize a vector<optional<Info>> (parallel to ids) as a JSON array, with
// null for not-found entries, matching the CLI batch-lookup semantics.
template <class Info, class Fn>
std::string SerializeLookup(const std::vector<std::optional<Info>>& results, Fn write_one) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  writer.StartArray();
  for (const auto& opt : results) {
    if (!opt) {
      writer.Null();
    } else {
      write_one(writer, *opt);
    }
  }
  writer.EndArray();
  return buffer.GetString();
}

// Common tail for the lookup tools: serialize results and report is_error when
// every id was missing (a wholly-failed lookup is a tool error; a partial hit
// is not).
template <class Info, class Fn>
std::pair<std::string, bool> RunLookup(const std::vector<std::optional<Info>>& results,
                                       Fn write_one) {
  std::string json = SerializeLookup(results, write_one);
  bool all_missing = true;
  for (const auto& opt : results) {
    if (opt) {
      all_missing = false;
      break;
    }
  }
  return {json, all_missing};
}

// A grouped lookup returns a group per id (an ident is reused across regions),
// so its result shape is vector<vector<Info>> rather than the optional-vector
// the other lookups use. Serialize as a JSON array parallel to `ids`, where each
// element is itself an array of the matches for that id (empty when unknown).
template <class Info, class Fn>
std::pair<std::string, bool> RunGroupedLookup(const std::vector<std::vector<Info>>& results,
                                              Fn write_one) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  writer.StartArray();
  bool all_empty = true;
  for (const auto& group : results) {
    writer.StartArray();
    for (const Info& x : group) {
      write_one(writer, x);
    }
    writer.EndArray();
    if (!group.empty()) {
      all_empty = false;
    }
  }
  writer.EndArray();
  return {buffer.GetString(), all_empty};
}

// Parse a string-array argument. Returns nullopt if missing or malformed.
std::optional<std::vector<std::string>> ParseIdList(const rapidjson::Value& args, const char* key) {
  if (!args.HasMember(key) || !args[key].IsArray()) {
    return std::nullopt;
  }
  const rapidjson::Value& arr = args[key];
  std::vector<std::string> ids;
  ids.reserve(arr.Size());
  for (const rapidjson::Value& v : arr.GetArray()) {
    if (!v.IsString()) {
      return std::nullopt;
    }
    ids.push_back(v.GetString());
  }
  return ids;
}

// Build a lookup Tool from its metadata plus a lookup function. The function
// takes the database and an id list and returns the parallel optional vector.
template <class Info, class Fn>
Tool MakeLookupTool(const char* name, const char* description, const char* schema, Fn lookup) {
  return Tool(name, description, ParseSchema(schema),
              [lookup](const rapidjson::Value& args,
                       const NavDatabase& db) -> std::pair<std::string, bool> {
                auto ids = ParseIdList(args, "ids");
                if (!ids) {
                  return {JsonError("ids (array of strings) is required"), true};
                }
                auto results = lookup(db, *ids);
                return RunLookup(results, [](auto& w, const Info& x) {
                  if constexpr (std::is_same_v<Info, bf::AirportInfo>) {
                    bf::WriteAirportJson(w, x);
                  } else if constexpr (std::is_same_v<Info, bf::AirportProcedures>) {
                    bf::WriteProceduresJson(w, x);
                  } else if constexpr (std::is_same_v<Info, bf::AirwayInfo>) {
                    bf::WriteAirwayJson(w, x);
                  }
                });
              });
}

// A grouped-lookup tool: an id maps to a group of matches (vector<vector<Info>>)
// rather than a single optional. Shares id-list parsing with the other lookups.
template <class Info, class Fn, class WriteFn>
Tool MakeGroupedLookupTool(const char* name, const char* description, const char* schema, Fn lookup,
                           WriteFn write_one) {
  return Tool(name, description, ParseSchema(schema),
              [lookup, write_one](const rapidjson::Value& args,
                                  const NavDatabase& db) -> std::pair<std::string, bool> {
                auto ids = ParseIdList(args, "ids");
                if (!ids) {
                  return {JsonError("ids (array of strings) is required"), true};
                }
                return RunGroupedLookup<Info>(lookup(db, *ids), write_one);
              });
}

// The find_routes handler.
std::pair<std::string, bool> FindRoutesHandler(const rapidjson::Value& args,
                                               const NavDatabase& db) {
  bf::RouteRequest request;
  if (!args.HasMember("departure") || !args["departure"].IsString() || !args.HasMember("arrival") ||
      !args["arrival"].IsString()) {
    return {JsonError("departure and arrival are required"), true};
  }
  request.departure = args["departure"].GetString();
  request.arrival = args["arrival"].GetString();
  // Altitude is an inclusive flight-level range. min_fl/max_fl may be given
  // together for a band, or either alone (the other defaults to it) for a
  // single level. Absent => no altitude/MORA filtering.
  {
    const bool has_min = args.HasMember("min_fl") && args["min_fl"].IsInt();
    const bool has_max = args.HasMember("max_fl") && args["max_fl"].IsInt();
    if (has_min || has_max) {
      const int min_fl = has_min ? args["min_fl"].GetInt() : args["max_fl"].GetInt();
      const int max_fl = has_max ? args["max_fl"].GetInt() : args["min_fl"].GetInt();
      if (min_fl > max_fl) {
        return {JsonError("min_fl must not exceed max_fl"), true};
      }
      request.altitude = bf::FlRange{min_fl, max_fl};
    }
  }
  if (args.HasMember("level") && args["level"].IsString()) {
    const std::string level = args["level"].GetString();
    if (level == "low") {
      request.level = bf::LevelPreference::kLow;
    } else if (level == "high") {
      request.level = bf::LevelPreference::kHigh;
    }
  }
  if (args.HasMember("k") && args["k"].IsInt()) {
    request.k = args["k"].GetInt();
  }
  // The schema declares minimum:1, but enforce it server-side too: Yen K-shortest
  // is undefined for k <= 0, and a non-positive value must not reach FindRoutes.
  if (request.k < 1) {
    return {JsonError("k must be a positive integer (>= 1)"), true};
  }
  if (args.HasMember("departure_runway") && args["departure_runway"].IsString()) {
    request.departure_runway = args["departure_runway"].GetString();
  }
  if (args.HasMember("arrival_runway") && args["arrival_runway"].IsString()) {
    request.arrival_runway = args["arrival_runway"].GetString();
  }
  if (args.HasMember("departure_sid") && args["departure_sid"].IsString()) {
    request.departure_sid = args["departure_sid"].GetString();
  }
  if (args.HasMember("arrival_star") && args["arrival_star"].IsString()) {
    request.arrival_star = args["arrival_star"].GetString();
  }
  if (auto v = ParseIdList(args, "avoid_waypoints")) {
    request.avoid_waypoints = std::move(*v);
  }
  if (auto v = ParseIdList(args, "avoid_airways")) {
    request.avoid_airways = std::move(*v);
  }
  if (args.HasMember("random_seed") && args["random_seed"].IsUint()) {
    request.random_seed = args["random_seed"].GetUint();
  }
  if (auto v = ParseIdList(args, "forced_points")) {
    request.forced_points = std::move(*v);
  }

  bf::Result<std::vector<bf::Route>> result = db.FindRoutes(request);
  if (!result) {
    // A failed route computation is a tool-level error: the LLM should see
    // isError=true rather than a 200-like payload carrying an "error" string.
    // The message may contain user-controlled strings (unknown departure, a bad
    // forced point token), so emit it through the Writer for auto-escaping.
    return {JsonError(result.error().message), true};
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(2);
  writer.StartArray();
  for (const bf::Route& route : result.value()) {
    bf::WriteRouteJson(writer, route);
  }
  writer.EndArray();
  return {buffer.GetString(), false};
}

// The lookup_procedure_legs handler: per-leg detail of one named procedure.
std::pair<std::string, bool> LookupProcedureLegsHandler(const rapidjson::Value& args,
                                                        const NavDatabase& db) {
  if (!args.HasMember("airport") || !args["airport"].IsString() || !args.HasMember("procedure") ||
      !args["procedure"].IsString()) {
    return {JsonError("airport and procedure are required"), true};
  }
  std::optional<bf::AirportProcedureDetail> detail =
      db.LookupProcedureDetail(args["airport"].GetString(), args["procedure"].GetString());
  if (!detail) {
    // Unknown airport, no CIFP data, or no procedure of that name: a tool error
    // so the caller sees isError rather than an empty success payload.
    return {JsonError("no procedure of that name at that airport"), true};
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  bf::WriteProcedureDetailJson(writer, *detail);
  return {buffer.GetString(), false};
}

// The parse_route handler: validate & expand a filed route string.
std::pair<std::string, bool> ParseRouteHandler(const rapidjson::Value& args,
                                               const NavDatabase& db) {
  if (!args.HasMember("route") || !args["route"].IsString()) {
    return {JsonError("route (string) is required"), true};
  }
  bf::Result<bf::Route> result = db.ParseRoute(args["route"].GetString());
  if (!result) {
    // The message names the offending token (user-controlled), so escape it.
    return {JsonError(result.error().message), true};
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(2);
  bf::WriteRouteJson(writer, result.value());
  return {buffer.GetString(), false};
}

std::vector<Tool> MakeTools() {
  // Build the tool list once for the server to own as a member. We emplace
  // (move) rather than use an initializer list, because Tool holds a
  // rapidjson::Document, which is movable but not copyable.
  std::vector<Tool> tools;
  tools.reserve(9);

  tools.emplace_back(
      "find_routes",
      "Find up to k candidate routes between two endpoints (airport ICAO or "
      "waypoint ident), honoring airway level and cruise-altitude constraints. "
      "Returns an ICAO filed-flight-plan style route string plus per-leg detail.",
      ParseSchema(
          R"({"type":"object","properties":{)"
          R"("departure":{"type":"string","description":"Departure airport ICAO or waypoint ident."},)"
          R"("arrival":{"type":"string","description":"Arrival airport ICAO or waypoint ident."},)"
          R"("min_fl":{"type":"integer","description":"Lower bound of the cruise flight-level range, in hundreds of feet (e.g. 300 for FL300). May be given alone for a single level. Setting min_fl and/or max_fl enables altitude/MORA constraint filtering."},)"
          R"("max_fl":{"type":"integer","description":"Upper bound of the cruise flight-level range, in hundreds of feet (e.g. 400 for FL400). May be given alone for a single level."},)"
          R"("level":{"type":"string","enum":["none","low","high"],"description":"Preferred airway level: none=no preference (default), low=prefer Victor low airways, high=prefer Jet high airways."},)"
          R"("k":{"type":"integer","minimum":1,"description":"Number of candidate routes to return (Yen K-shortest). Defaults to 1."},)"
          R"("departure_runway":{"type":"string","description":"Restrict the SID to this departure runway, e.g. RW31L. Empty=any."},)"
          R"("arrival_runway":{"type":"string","description":"Restrict the STAR to this arrival runway, e.g. RW25L. Empty=any."},)"
          R"("departure_sid":{"type":"string","description":"Pin a specific SID by name, e.g. DEEZZ5 or DEEZZ5.TOWIN. Empty=auto."},)"
          R"("arrival_star":{"type":"string","description":"Pin a specific STAR by name, e.g. LENDY6 or LENDY6.HAAYS. Empty=auto."},)"
          R"("avoid_waypoints":{"type":"array","items":{"type":"string"},"description":"Waypoints to route around, each an ident (BOTON) or IDENT/REGION (BOTON/LF). A bare ident avoids all regions' matches."},)"
          R"("avoid_airways":{"type":"array","items":{"type":"string"},"description":"Airway designators to route around, e.g. J60. Also blocks concurrency segments recorded as J60-V123."},)"
          R"("random_seed":{"type":"integer","minimum":0,"description":"Seed for reproducible route diversity. The same seed always yields the same route; different seeds explore alternatives. Omit for the plain optimal route."},)"
          R"("forced_points":{"type":"array","items":{"type":"string"},"description":"Ordered waypoints the route must pass through (via points), each an ident (PSB) or IDENT/REGION (PSB/K6). The response echoes them resolved as IDENT/REGION."}},)"
          R"("required":["departure","arrival"]})"),
      FindRoutesHandler);

  tools.emplace_back(
      "parse_route",
      "Validate and expand a filed-flight-plan route string (the reverse of "
      "find_routes). Given \"[DEP] [SID] FIX (AWY FIX | DCT FIX)* [STAR] [ARR]\", "
      "checks that each airway connects its bracketing fixes, expands airways to "
      "their intermediate points, totals the distance, and returns the resolved "
      "route. Errors name the offending token when the route is invalid.",
      ParseSchema(
          R"({"type":"object","properties":{)"
          R"("route":{"type":"string","description":"Filed route string, e.g. 'KJFK DEEZZ5 CANDR J60 PSB ... KLAX'."}},)"
          R"("required":["route"]})"),
      ParseRouteHandler);

  tools.push_back(MakeGroupedLookupTool<bf::WaypointInfo>(
      "lookup_waypoints",
      "Look up waypoints / navaids by ident. An ident is reused across regions, "
      "so each id returns a group of matches (ident, region, coordinate, kind, "
      "on-network flag). Batch: one group per id, empty when not found.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more waypoint idents to look up. Each result is a group parallel to this list."}},)"
      R"("required":["ids"]})",
      [](const NavDatabase& db, const std::vector<std::string>& ids) {
        return db.LookupWaypoints(ids);
      },
      [](auto& w, const bf::WaypointInfo& x) { bf::WriteWaypointJson(w, x); }));

  tools.push_back(MakeLookupTool<bf::AirportInfo>(
      "lookup_airports",
      "Look up airports by ICAO code. Returns coordinates, elevation, and "
      "whether the airport publishes terminal procedures. Batch: one result "
      "per id, null when not found.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more airport ICAO codes. Results are parallel to this list."}},)"
      R"("required":["ids"]})",
      [](const NavDatabase& db, const std::vector<std::string>& ids) {
        return db.LookupAirports(ids);
      }));

  tools.push_back(MakeLookupTool<bf::AirportProcedures>(
      "lookup_procedures",
      "Look up published terminal procedures (SID/STAR/approach) by airport "
      "ICAO. Returns a summary per procedure (type, name, transition, runway). "
      "Batch: one result per id, null when unknown or no CIFP data.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more airport ICAO codes. Results are parallel to this list."}},)"
      R"("required":["ids"]})",
      [](const NavDatabase& db, const std::vector<std::string>& ids) {
        return db.LookupProcedures(ids);
      }));

  tools.emplace_back(
      "lookup_procedure_legs",
      "Look up the per-leg detail of one named terminal procedure at an airport. "
      "Given an airport ICAO and a procedure name (e.g. DEEZZ5), returns every "
      "transition of that procedure with its ordered legs: fix, path terminator, "
      "magnetic course, distance, altitude constraint, RNP, turn direction, and "
      "speed limit. Errors when the airport is unknown or publishes no such procedure.",
      ParseSchema(
          R"({"type":"object","properties":{)"
          R"("airport":{"type":"string","description":"Airport ICAO code, e.g. KJFK."},)"
          R"("procedure":{"type":"string","description":"Published procedure name, e.g. DEEZZ5 or LENDY6."}},)"
          R"("required":["airport","procedure"]})"),
      LookupProcedureLegsHandler);

  tools.push_back(MakeLookupTool<bf::AirwayInfo>(
      "lookup_airways",
      "Look up airways by designator (e.g. Y28). Returns every directed "
      "segment carrying that name with distances and flight-level bounds. "
      "Batch: one result per id, null when no segment uses the name.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more airway designators. Results are parallel to this list."}},)"
      R"("required":["ids"]})",
      [](const NavDatabase& db, const std::vector<std::string>& ids) {
        return db.LookupAirways(ids);
      }));

  tools.push_back(MakeGroupedLookupTool<bf::NavaidDetailInfo>(
      "lookup_navaid_detail",
      "Look up detailed radio-navaid attributes by ident (frequency, service "
      "range, elevation, and heading/variation). An ident is reused across "
      "regions, so each id returns a group of matches. freq_raw is kHz for NDBs "
      "and MHz*100 for VOR/DME/ILS. Requires a detail cache; empty otherwise.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more navaid idents. Each result is a group parallel to this list."}},)"
      R"("required":["ids"]})",
      [](const NavDatabase& db, const std::vector<std::string>& ids) {
        return db.LookupNavaidDetails(ids);
      },
      [](auto& w, const bf::NavaidDetailInfo& x) { bf::WriteNavaidDetailJson(w, x); }));

  tools.push_back(MakeGroupedLookupTool<bf::HoldInfo>(
      "lookup_holds",
      "Look up holding patterns by fix ident (inbound course, outbound leg "
      "time/distance, turn direction, altitude window, speed limit). Each id "
      "returns a group of holds across all regions/airports at that fix. "
      "Requires a detail cache; empty otherwise.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more hold fix idents. Each result is a group parallel to this list."}},)"
      R"("required":["ids"]})",
      [](const NavDatabase& db, const std::vector<std::string>& ids) {
        return db.LookupHolds(ids);
      },
      [](auto& w, const bf::HoldInfo& x) { bf::WriteHoldJson(w, x); }));

  return tools;
}

}  // namespace bf::mcp
