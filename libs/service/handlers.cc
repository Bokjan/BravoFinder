// SPDX-License-Identifier: MIT
// handlers.cc — the JSON-args adapter layer over the typed query entries.
//
// Each handler parses the request "arguments" object (the wire shape the MCP and
// HTTP transports ship) into typed values and delegates to a queries.h entry
// with OutputFormat::kJson. The engine call, rendering, and status mapping live
// in queries.cc / render.cc, shared with the CLI. Adding a handler means adding
// one entry to MakeHandlers().

#include "handlers.h"

#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/constraints/airway_rule_constraint.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "queries.h"
#include "rapidjson/document.h"
#include "render.h"

namespace bf::service {

namespace {

// Contract: a handler must ignore any key in `args` it does not recognize. The
// MCP transport (apps/mcp) forwards the server-level "cycle" selector through to
// the handler as part of the args object even though "cycle" is not declared in
// any tool's JSON-Schema; a handler that rejected unknown keys would wrongly
// fail every MCP call that pins a cycle, so strict unknown-key validation is
// out of bounds here.
//
// HTTP-style status codes the adapters report on a bad request (see
// HandlerResult in the header for the 400/404/422 semantics; 404/422 are
// produced inside the typed entries).
constexpr int kBadRequest = 400;

// Max entries allowed in any request ID array (forced_points, avoid_waypoints,
// avoid_airways, lookup ids). The only other size bound is the 1 MiB body cap,
// under which a single request could still ship tens of thousands of short
// idents; each drives graph lookups / per-edge ban checks, an asymmetric-cost
// vector. 256 far exceeds any real "force via / avoid these" list.
constexpr size_t kMaxIdListSize = 256;

// Max accepted flight level (hundreds of feet). GetInt() otherwise accepts values
// up to ~2.1e9, which are physically meaningless; FL600 (60,000 ft) is already
// above any civil cruise altitude.
constexpr int kMaxFl = 600;

// Parse a string-array argument. Returns nullopt if the member is absent OR
// malformed (not an array, a non-string element, or more than kMaxIdListSize
// entries). Callers that need to tell "absent" (skip) from "present but bad"
// (reject) check HasMember themselves before deciding.
std::optional<std::vector<std::string>> ParseIdList(const rapidjson::Value& args, const char* key) {
  if (!args.HasMember(key) || !args[key].IsArray()) {
    return std::nullopt;
  }
  const rapidjson::Value& arr = args[key];
  if (arr.Size() > kMaxIdListSize) {
    return std::nullopt;  // over the cap: treat as malformed so callers reject it
  }
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

// Parse one string-array member of a rule object, with the same "absent or
// malformed" collapse as ParseIdList. An absent member means "match everything"
// for that side of the rule, which is an empty vector -- so absence and an empty
// array are deliberately indistinguishable here. Returns false only for a member
// that is present and genuinely malformed.
bool ParseRuleStringList(const rapidjson::Value& rule, const char* key,
                         std::vector<std::string>& out) {
  if (!rule.HasMember(key)) {
    return true;  // absent => match everything on this side
  }
  if (!rule[key].IsArray() || rule[key].Size() > kMaxIdListSize) {
    return false;
  }
  for (const rapidjson::Value& v : rule[key].GetArray()) {
    if (!v.IsString()) {
      return false;
    }
    out.emplace_back(v.GetString());
  }
  return true;
}

// Parse the airway_rules array into structured AirwayRules. Returns an error
// message on any malformed entry (the caller turns it into a 400), or nullopt on
// success. Unlike the flat ID lists, each element is an object, so the shape is
// validated field by field.
//
// The cap is kMaxRules (32), not kMaxIdListSize (256): resolving a rule costs
// O(V + names) work at query time, so 256 rules would be tens of millions of
// comparisons. It is not a real limit either way -- both sides of a rule are lists
// sharing one rule slot, so "block these 200 airways" is a single rule.
std::optional<std::string> ParseAirwayRules(const rapidjson::Value& args,
                                            std::vector<bf::AirwayRule>& out) {
  if (!args.HasMember("airway_rules")) {
    return std::nullopt;
  }
  if (!args["airway_rules"].IsArray()) {
    return "airway_rules must be an array of rule objects";
  }
  const rapidjson::Value& arr = args["airway_rules"];
  if (arr.Size() > bf::AirwayRuleConstraint::kMaxRules) {
    return "airway_rules must hold at most 32 rules (one rule may list any number "
           "of regions and designators)";
  }
  for (const rapidjson::Value& rule : arr.GetArray()) {
    if (!rule.IsObject()) {
      return "each airway_rules entry must be an object";
    }
    bf::AirwayRule parsed;
    if (!ParseRuleStringList(rule, "region_prefixes", parsed.region_prefixes)) {
      return "region_prefixes must be an array of at most 256 strings";
    }
    if (!ParseRuleStringList(rule, "designators", parsed.designators)) {
      return "designators must be an array of at most 256 strings";
    }
    if (rule.HasMember("match")) {
      if (!rule["match"].IsString()) {
        return "match must be \"exact\" or \"prefix\"";
      }
      const std::string_view match = rule["match"].GetString();
      if (match == "exact") {
        parsed.match = bf::AirwayRule::Match::kExact;
      } else if (match == "prefix") {
        parsed.match = bf::AirwayRule::Match::kPrefix;
      } else {
        return "match must be \"exact\" or \"prefix\"";
      }
    }
    if (rule.HasMember("action")) {
      if (!rule["action"].IsString()) {
        return "action must be \"block\" or \"penalize\"";
      }
      const std::string_view action = rule["action"].GetString();
      if (action == "block") {
        parsed.action = bf::AirwayRule::Action::kBlock;
      } else if (action == "penalize") {
        parsed.action = bf::AirwayRule::Action::kPenalize;
      } else {
        return "action must be \"block\" or \"penalize\"";
      }
    }
    if (rule.HasMember("penalty_fraction")) {
      if (!rule["penalty_fraction"].IsNumber()) {
        return "penalty_fraction must be a number >= 0";
      }
      const double fraction = rule["penalty_fraction"].GetDouble();
      // Reject negatives and non-finite values: a negative penalty would break the
      // search heuristic's admissibility. No upper bound -- a large fraction is a
      // legitimate "almost block, but keep the graph connected".
      if (!std::isfinite(fraction) || fraction < 0.0) {
        return "penalty_fraction must be a finite number >= 0";
      }
      parsed.penalty_fraction = fraction;
    }
    out.push_back(std::move(parsed));
  }
  return std::nullopt;
}

// Build a batch-lookup handler from a typed LookupX entry: parse the ids array
// (400 on a malformed/missing list), then delegate to the entry in JSON.
template <class Fn>
QueryHandler MakeLookupAdapter(Fn fn) {
  return [fn](const rapidjson::Value& args, const NavDatabase& db) -> HandlerResult {
    auto ids = ParseIdList(args, "ids");
    if (!ids) {
      return {JsonError("ids (array of at most 256 strings) is required"), kBadRequest};
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
      // Bound the top of the range: an absurdly high FL is physically
      // meaningless and only invites overflow-adjacent inputs. min_fl <= max_fl
      // is already enforced, so capping max_fl bounds both.
      if (max_fl > kMaxFl) {
        return {JsonError("min_fl and max_fl must not exceed 600"), kBadRequest};
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
    // Any other string (including "none", or an invalid "medium"/miscased value)
    // intentionally leaves request.level at its kNone default. MCP clients get a
    // first line of defense from the schema enum; HTTP has none, so silently
    // mapping an unknown level to the no-preference default is the accepted
    // fallback rather than a hard error.
  }
  if (args.HasMember("k")) {
    // A present-but-non-integer k (e.g. 3.5, or the string "3") must be rejected,
    // not silently dropped to the default: the caller clearly meant to set k, and
    // ignoring it would hand back one route where several were asked for.
    if (!args["k"].IsInt()) {
      return {JsonError("k must be an integer"), kBadRequest};
    }
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
  // For the optional ID lists, tell "absent" (skip) from "present but bad"
  // (reject): a present list that is malformed or over the 256 cap is a 400, not
  // a silent no-op that would drop a user's avoid/force intent.
  if (args.HasMember("avoid_waypoints")) {
    auto v = ParseIdList(args, "avoid_waypoints");
    if (!v) {
      return {JsonError("avoid_waypoints must be an array of at most 256 strings"), kBadRequest};
    }
    request.avoid_waypoints = std::move(*v);
  }
  if (std::optional<std::string> err = ParseAirwayRules(args, request.airway_rules)) {
    return {JsonError(*err), kBadRequest};
  }
  if (args.HasMember("random_seed") && args["random_seed"].IsUint()) {
    request.random_seed = args["random_seed"].GetUint();
  }
  if (args.HasMember("forced_points")) {
    auto v = ParseIdList(args, "forced_points");
    if (!v) {
      return {JsonError("forced_points must be an array of at most 256 strings"), kBadRequest};
    }
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
