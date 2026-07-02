// tools.cc — the capabilities exposed by bf-mcp-stdio, as self-describing Tools.
//
// Each Tool binds its MCP name, description, JSON-Schema input descriptor, and
// handler in one place. Handlers are pure with respect to server state: they
// receive the request arguments and a read-only NavDatabase, and return
// {json_text, is_error}. Adding a tool means adding one entry to AllTools();
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
#include "rapidjson_document.h"
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

// The waypoint lookup returns a group per id (an ident is reused across
// regions), so its result shape is vector<vector<WaypointInfo>> rather than the
// optional-vector the other lookups use. Serialize as a JSON array parallel to
// `ids`, where each element is itself an array of the region matches for that
// id (empty when the ident is unknown or names an airport).
std::pair<std::string, bool> RunWaypointLookup(
    const std::vector<std::vector<bf::WaypointInfo>>& results) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  writer.StartArray();
  bool all_empty = true;
  for (const auto& group : results) {
    writer.StartArray();
    for (const bf::WaypointInfo& w : group) {
      bf::WriteWaypointJson(writer, w);
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
                  return {R"({"error":"ids (array of strings) is required"})", true};
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

// The waypoint lookup is special: an ident maps to a group of region matches
// (vector<vector<WaypointInfo>>), not a single optional. This tool handles that
// shape while keeping the id-list parsing shared with the other lookups.
Tool MakeWaypointLookupTool(const char* name, const char* description, const char* schema) {
  return Tool(
      name, description, ParseSchema(schema),
      [](const rapidjson::Value& args, const NavDatabase& db) -> std::pair<std::string, bool> {
        auto ids = ParseIdList(args, "ids");
        if (!ids) {
          return {R"({"error":"ids (array of strings) is required"})", true};
        }
        return RunWaypointLookup(db.LookupWaypoints(*ids));
      });
}

// The find_routes handler.
std::pair<std::string, bool> FindRoutesHandler(const rapidjson::Value& args,
                                               const NavDatabase& db) {
  bf::RouteRequest request;
  if (!args.HasMember("departure") || !args["departure"].IsString() || !args.HasMember("arrival") ||
      !args["arrival"].IsString()) {
    return {R"({"error":"departure and arrival are required"})", true};
  }
  request.departure = args["departure"].GetString();
  request.arrival = args["arrival"].GetString();
  if (args.HasMember("cruise_fl") && args["cruise_fl"].IsInt()) {
    request.cruise_fl = args["cruise_fl"].GetInt();
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

  bf::Result<std::vector<bf::Route>> result = db.FindRoutes(request);
  if (!result) {
    // A failed route computation is a tool-level error: the LLM should see
    // isError=true rather than a 200-like payload carrying an "error" string.
    return {R"({"error":")" + result.error().message + R"("})", true};
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

const std::vector<Tool>& AllTools() {
  // Lazily build the tool list once. We emplace (move) rather than use an
  // initializer list, because Tool holds a rapidjson::Document, which is
  // movable but not copyable.
  static std::vector<Tool> tools;
  if (!tools.empty()) {
    return tools;
  }
  tools.reserve(5);

  tools.emplace_back(
      "find_routes",
      "Find up to k candidate routes between two endpoints (airport ICAO or "
      "waypoint ident), honoring airway level and cruise-altitude constraints. "
      "Returns an ICAO filed-flight-plan style route string plus per-leg detail.",
      ParseSchema(
          R"({"type":"object","properties":{)"
          R"("departure":{"type":"string","description":"Departure airport ICAO or waypoint ident."},)"
          R"("arrival":{"type":"string","description":"Arrival airport ICAO or waypoint ident."},)"
          R"("cruise_fl":{"type":"integer","description":"Cruise flight level in hundreds of feet, e.g. 350 for FL350. Setting it enables altitude/MORA constraint filtering."},)"
          R"("level":{"type":"string","enum":["none","low","high"],"description":"Preferred airway level: none=no preference (default), low=prefer Victor low airways, high=prefer Jet high airways."},)"
          R"("k":{"type":"integer","minimum":1,"description":"Number of candidate routes to return (Yen K-shortest). Defaults to 1."},)"
          R"("departure_runway":{"type":"string","description":"Restrict the SID to this departure runway, e.g. RW31L. Empty=any."},)"
          R"("arrival_runway":{"type":"string","description":"Restrict the STAR to this arrival runway, e.g. RW25L. Empty=any."},)"
          R"("departure_sid":{"type":"string","description":"Pin a specific SID by name, e.g. DEEZZ5 or DEEZZ5.TOWIN. Empty=auto."},)"
          R"("arrival_star":{"type":"string","description":"Pin a specific STAR by name, e.g. LENDY6 or LENDY6.HAAYS. Empty=auto."}},)"
          R"("required":["departure","arrival"]})"),
      FindRoutesHandler);

  tools.push_back(MakeWaypointLookupTool(
      "lookup_waypoints",
      "Look up waypoints / navaids by ident. An ident is reused across regions, "
      "so each id returns a group of matches (ident, region, coordinate, kind, "
      "on-network flag). Batch: one group per id, empty when not found.",
      R"({"type":"object","properties":{)"
      R"("ids":{"type":"array","items":{"type":"string"},"description":"One or more waypoint idents to look up. Each result is a group parallel to this list."}},)"
      R"("required":["ids"]})"));

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

  return tools;
}

}  // namespace bf::mcp
