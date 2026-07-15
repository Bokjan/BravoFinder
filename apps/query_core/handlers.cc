// handlers.cc — the transport-neutral query handlers.
//
// Each handler receives the request arguments and a read-only NavDatabase and
// returns a HandlerResult {json_body, status}. The logic (argument parsing,
// database calls, JSON serialization) is shared verbatim by every transport;
// only the status code differs from a plain 200 on failure. Adding a handler
// means adding one entry to MakeHandlers().

#include "handlers.h"

#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/query/query_json.h"
#include "core/routing/route.h"
#include "core/routing/route_json.h"
#include "core/routing/route_request.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace bf::service {

// Build an error JSON payload `{"error":"<message>"}` using RapidJSON's Writer
// so the message is auto-escaped. Error messages may carry user-controlled
// strings (an unknown departure airport, a bad route token, an unknown tool
// name), and a hand-rolled `R"({"error":")" + msg + R"("})"` concatenation
// would let a quote/backslash in the message break the JSON frame.
std::string JsonError(const std::string& message) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("error");
  writer.String(message.c_str(), static_cast<unsigned>(message.size()));
  writer.EndObject();
  return buffer.GetString();
}

namespace {

// HTTP-style status codes the handlers report. 400: the request was malformed
// (missing/invalid arguments). 404: nothing matched (all ids missing, no such
// procedure). 422: the request was well-formed but could not be satisfied (no
// route, a bad route token) -- distinct from 400 so callers can tell "you sent
// it wrong" from "you sent it right but there is no answer".
constexpr int kOk = 200;
constexpr int kBadRequest = 400;
constexpr int kNotFound = 404;
constexpr int kUnprocessable = 422;

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

// Common tail for the lookup handlers: serialize results and report 404 when
// every id was missing (a wholly-failed lookup is not-found; a partial hit is
// 200).
template <class Info, class Fn>
HandlerResult RunLookup(const std::vector<std::optional<Info>>& results, Fn write_one) {
  std::string json = SerializeLookup(results, write_one);
  bool all_missing = true;
  for (const auto& opt : results) {
    if (opt) {
      all_missing = false;
      break;
    }
  }
  return {std::move(json), all_missing ? kNotFound : kOk};
}

// A grouped lookup returns a group per id (an ident is reused across regions),
// so its result shape is vector<vector<Info>> rather than the optional-vector
// the other lookups use. Serialize as a JSON array parallel to `ids`, where each
// element is itself an array of the matches for that id (empty when unknown).
template <class Info, class Fn>
HandlerResult RunGroupedLookup(const std::vector<std::vector<Info>>& results, Fn write_one) {
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
  return {buffer.GetString(), all_empty ? kNotFound : kOk};
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

// Build a batch-lookup handler from a lookup function. The function takes the
// database and an id list and returns the parallel optional vector.
template <class Info, class Fn>
QueryHandler MakeLookupHandler(Fn lookup) {
  return [lookup](const rapidjson::Value& args, const NavDatabase& db) -> HandlerResult {
    auto ids = ParseIdList(args, "ids");
    if (!ids) {
      return {JsonError("ids (array of strings) is required"), kBadRequest};
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
  };
}

// A grouped-lookup handler: an id maps to a group of matches (vector<vector<Info>>)
// rather than a single optional. Shares id-list parsing with the other lookups.
template <class Info, class Fn, class WriteFn>
QueryHandler MakeGroupedLookupHandler(Fn lookup, WriteFn write_one) {
  return [lookup, write_one](const rapidjson::Value& args, const NavDatabase& db) -> HandlerResult {
    auto ids = ParseIdList(args, "ids");
    if (!ids) {
      return {JsonError("ids (array of strings) is required"), kBadRequest};
    }
    return RunGroupedLookup<Info>(lookup(db, *ids), write_one);
  };
}

// The find_routes handler.
HandlerResult FindRoutesHandler(const rapidjson::Value& args, const NavDatabase& db) {
  bf::RouteRequest request;
  if (!args.HasMember("departure") || !args["departure"].IsString() || !args.HasMember("arrival") ||
      !args["arrival"].IsString()) {
    return {JsonError("departure and arrival are required"), kBadRequest};
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
        return {JsonError("min_fl must not exceed max_fl"), kBadRequest};
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
    return {JsonError("k must be a positive integer (>= 1)"), kBadRequest};
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
    // A failed route computation is a semantic failure (422): the request was
    // well-formed but no route satisfies it, or an endpoint is unknown. The
    // message may contain user-controlled strings (unknown departure, a bad
    // forced point token), so emit it through the Writer for auto-escaping.
    return {JsonError(result.error().message), kUnprocessable};
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(2);
  writer.StartArray();
  for (const bf::Route& route : result.value()) {
    bf::WriteRouteJson(writer, route);
  }
  writer.EndArray();
  return {buffer.GetString(), kOk};
}

// The lookup_procedure_legs handler: per-leg detail of one named procedure.
HandlerResult LookupProcedureLegsHandler(const rapidjson::Value& args, const NavDatabase& db) {
  if (!args.HasMember("airport") || !args["airport"].IsString() || !args.HasMember("procedure") ||
      !args["procedure"].IsString()) {
    return {JsonError("airport and procedure are required"), kBadRequest};
  }
  std::optional<bf::AirportProcedureDetail> detail =
      db.LookupProcedureDetail(args["airport"].GetString(), args["procedure"].GetString());
  if (!detail) {
    // Unknown airport, no CIFP data, or no procedure of that name: nothing
    // matched, so 404 rather than an empty success payload.
    return {JsonError("no procedure of that name at that airport"), kNotFound};
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  bf::WriteProcedureDetailJson(writer, *detail);
  return {buffer.GetString(), kOk};
}

// The parse_route handler: validate & expand a filed route string.
HandlerResult ParseRouteHandler(const rapidjson::Value& args, const NavDatabase& db) {
  if (!args.HasMember("route") || !args["route"].IsString()) {
    return {JsonError("route (string) is required"), kBadRequest};
  }
  bf::Result<bf::Route> result = db.ParseRoute(args["route"].GetString());
  if (!result) {
    // A parse failure is a semantic failure (422): the string was given but
    // does not form a valid route. The message names the offending token
    // (user-controlled), so escape it.
    return {JsonError(result.error().message), kUnprocessable};
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(2);
  bf::WriteRouteJson(writer, result.value());
  return {buffer.GetString(), kOk};
}

}  // namespace

std::vector<NamedHandler> MakeHandlers() {
  // Built once for the caller to own. The order is the display order the MCP
  // transport presents (find_routes, parse_route, then the batch lookups); the
  // HTTP transport keys off the names, not the order.
  std::vector<NamedHandler> handlers;
  handlers.reserve(9);

  handlers.push_back({"find_routes", FindRoutesHandler});
  handlers.push_back({"parse_route", ParseRouteHandler});
  handlers.push_back(
      {"lookup_waypoints",
       MakeGroupedLookupHandler<bf::WaypointInfo>(
           [](const NavDatabase& db, const std::vector<std::string>& ids) {
             return db.LookupWaypoints(ids);
           },
           [](auto& w, const bf::WaypointInfo& x) { bf::WriteWaypointJson(w, x); })});
  handlers.push_back(
      {"lookup_airports", MakeLookupHandler<bf::AirportInfo>(
                              [](const NavDatabase& db, const std::vector<std::string>& ids) {
                                return db.LookupAirports(ids);
                              })});
  handlers.push_back(
      {"lookup_procedures", MakeLookupHandler<bf::AirportProcedures>(
                                [](const NavDatabase& db, const std::vector<std::string>& ids) {
                                  return db.LookupProcedures(ids);
                                })});
  handlers.push_back({"lookup_procedure_legs", LookupProcedureLegsHandler});
  handlers.push_back(
      {"lookup_airways", MakeLookupHandler<bf::AirwayInfo>(
                             [](const NavDatabase& db, const std::vector<std::string>& ids) {
                               return db.LookupAirways(ids);
                             })});
  handlers.push_back(
      {"lookup_navaid_detail",
       MakeGroupedLookupHandler<bf::NavaidDetailInfo>(
           [](const NavDatabase& db, const std::vector<std::string>& ids) {
             return db.LookupNavaidDetails(ids);
           },
           [](auto& w, const bf::NavaidDetailInfo& x) { bf::WriteNavaidDetailJson(w, x); })});
  handlers.push_back(
      {"lookup_holds", MakeGroupedLookupHandler<bf::HoldInfo>(
                           [](const NavDatabase& db, const std::vector<std::string>& ids) {
                             return db.LookupHolds(ids);
                           },
                           [](auto& w, const bf::HoldInfo& x) { bf::WriteHoldJson(w, x); })});

  return handlers;
}

}  // namespace bf::service
