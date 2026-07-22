// handlers.cc — the JSON-args adapter layer over the typed query entries.
//
// Each handler parses the request "arguments" object (the wire shape the MCP and
// HTTP transports ship) into typed values and delegates to a queries.h entry
// with OutputFormat::kJson. The engine call, rendering, and status mapping live
// in queries.cc / render.cc, shared with the CLI. Adding a handler means adding
// one entry to MakeHandlers().

#include "handlers.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "queries.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "render.h"

namespace bf::service {

// Build `{"error":"<message>"}` via RapidJSON's Writer so the message is
// auto-escaped; hand-rolling it is unsafe (see JsonError in the header). Shared
// by the adapters here and by render.cc's RenderError.
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

// HTTP-style status codes the adapters report on a bad request (see
// HandlerResult in the header for the 400/404/422 semantics; 404/422 are
// produced inside the typed entries).
constexpr int kBadRequest = 400;

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

// Build a batch-lookup handler from a typed LookupX entry: parse the ids array
// (400 on a malformed/missing list), then delegate to the entry in JSON.
template <class Fn>
QueryHandler MakeLookupAdapter(Fn fn) {
  return [fn](const rapidjson::Value& args, const NavDatabase& db) -> HandlerResult {
    auto ids = ParseIdList(args, "ids");
    if (!ids) {
      return {JsonError("ids (array of strings) is required"), kBadRequest};
    }
    return fn(db, *ids, OutputFormat::kJson);
  };
}

// The find_routes handler: parse the args object into a RouteRequest (with the
// server-side validation the typed entry does not redo), then delegate.
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
      // Flight levels are non-negative (hundreds of feet). A negative bound is
      // physically meaningless and would make the altitude-band and MORA
      // constraints reject every edge with recorded data, silently yielding
      // "no route" rather than an error. Reject it up front. (min_fl <= max_fl
      // is already enforced, so this bounds both.)
      if (min_fl < 0) {
        return {JsonError("min_fl and max_fl must be non-negative"), kBadRequest};
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
  // Cap k as well: each extra path costs a full spur A* search, so an unbounded k
  // (e.g. 2e9) would pin a worker for a long time and let a few requests exhaust
  // the threadpool. 15 far exceeds any real "give me alternatives" use. (The CLI
  // does not enforce this cap -- it is a server-protection concern -- so the cap
  // lives here in the adapter, not in the typed entry.)
  constexpr int kMaxK = 15;
  if (request.k > kMaxK) {
    return {JsonError("k must not exceed 15"), kBadRequest};
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

  return FindRoutes(db, request, OutputFormat::kJson);
}

// The lookup_procedure_legs handler: per-leg detail of one named procedure.
HandlerResult LookupProcedureLegsHandler(const rapidjson::Value& args, const NavDatabase& db) {
  if (!args.HasMember("airport") || !args["airport"].IsString() || !args.HasMember("procedure") ||
      !args["procedure"].IsString()) {
    return {JsonError("airport and procedure are required"), kBadRequest};
  }
  return LookupProcedureLegs(db, args["airport"].GetString(), args["procedure"].GetString(),
                             OutputFormat::kJson);
}

// The parse_route handler: validate & expand a filed route string.
HandlerResult ParseRouteHandler(const rapidjson::Value& args, const NavDatabase& db) {
  if (!args.HasMember("route") || !args["route"].IsString()) {
    return {JsonError("route (string) is required"), kBadRequest};
  }
  return ParseRoute(db, args["route"].GetString(), OutputFormat::kJson);
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
  handlers.push_back({"lookup_waypoints", MakeLookupAdapter(LookupWaypoints)});
  handlers.push_back({"lookup_airports", MakeLookupAdapter(LookupAirports)});
  handlers.push_back({"lookup_procedures", MakeLookupAdapter(LookupProcedures)});
  handlers.push_back({"lookup_procedure_legs", LookupProcedureLegsHandler});
  handlers.push_back({"lookup_airways", MakeLookupAdapter(LookupAirways)});
  handlers.push_back({"lookup_navaid_detail", MakeLookupAdapter(LookupNavaidDetails)});
  handlers.push_back({"lookup_holds", MakeLookupAdapter(LookupHolds)});

  return handlers;
}

}  // namespace bf::service
