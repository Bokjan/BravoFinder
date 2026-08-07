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
#include <format>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "api_keys.h"
#include "core/base/string_util.h"
#include "core/constraints/airway_rule_constraint.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "http_status.h"
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

using bf::http_server::kStatusBadRequest;

// rapidjson::HasMember / operator[] need a null-terminated C string; every
// api_keys.h string_view is backed by a string literal, so .data() is safe.
bool HasKey(const rapidjson::Value& o, std::string_view key) { return o.HasMember(key.data()); }

// Parse a string-array argument. Returns nullopt if the member is absent OR
// malformed (not an array, a non-string element, or more than kMaxIdListSize
// entries). Callers that need to tell "absent" (skip) from "present but bad"
// (reject) check HasKey themselves before deciding.
std::optional<std::vector<std::string>> ParseIdList(const rapidjson::Value& args,
                                                    std::string_view key) {
  if (!HasKey(args, key) || !args[key.data()].IsArray()) {
    return std::nullopt;
  }
  const rapidjson::Value& arr = args[key.data()];
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
// that is present and genuinely malformed. Both region and designator entries
// are upper-cased, since the navigation data stores them upper-case and the rest
// of the query layer already normalizes user input (see ToUpper).
bool ParseRuleStringList(const rapidjson::Value& rule, std::string_view key,
                         std::vector<std::string>& out) {
  if (!HasKey(rule, key)) {
    return true;  // absent => match everything on this side
  }
  if (!rule[key.data()].IsArray() || rule[key.data()].Size() > kMaxIdListSize) {
    return false;
  }
  for (const rapidjson::Value& v : rule[key.data()].GetArray()) {
    if (!v.IsString()) {
      return false;
    }
    out.emplace_back(ToUpper(v.GetString()));
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
  if (!HasKey(args, kKeyAirwayRules)) {
    return std::nullopt;
  }
  if (!args[kKeyAirwayRules.data()].IsArray()) {
    return std::format("{} must be an array of rule objects", kKeyAirwayRules);
  }
  const rapidjson::Value& arr = args[kKeyAirwayRules.data()];
  if (arr.Size() > bf::AirwayRuleConstraint::kMaxRules) {
    return std::format(
        "{} must hold at most {} rules (one rule may list any number "
        "of regions and designators)",
        kKeyAirwayRules, bf::AirwayRuleConstraint::kMaxRules);
  }
  for (const rapidjson::Value& rule : arr.GetArray()) {
    if (!rule.IsObject()) {
      return std::format("each {} entry must be an object", kKeyAirwayRules);
    }
    bf::AirwayRule parsed;
    if (!ParseRuleStringList(rule, kKeyRegionPrefixes, parsed.region_prefixes)) {
      return std::format("{} must be an array of at most {} strings", kKeyRegionPrefixes,
                         kMaxIdListSize);
    }
    if (!ParseRuleStringList(rule, kKeyDesignators, parsed.designators)) {
      return std::format("{} must be an array of at most {} strings", kKeyDesignators,
                         kMaxIdListSize);
    }
    if (HasKey(rule, kKeyMatch)) {
      if (!rule[kKeyMatch.data()].IsString()) {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyMatch, kMatchExact, kMatchPrefix);
      }
      const std::string_view match = rule[kKeyMatch.data()].GetString();
      if (match == kMatchExact) {
        parsed.match = bf::AirwayRule::Match::kExact;
      } else if (match == kMatchPrefix) {
        parsed.match = bf::AirwayRule::Match::kPrefix;
      } else {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyMatch, kMatchExact, kMatchPrefix);
      }
    }
    if (HasKey(rule, kKeyAction)) {
      if (!rule[kKeyAction.data()].IsString()) {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyAction, kActionBlock,
                           kActionPenalize);
      }
      const std::string_view action = rule[kKeyAction.data()].GetString();
      if (action == kActionBlock) {
        parsed.action = bf::AirwayRule::Action::kBlock;
      } else if (action == kActionPenalize) {
        parsed.action = bf::AirwayRule::Action::kPenalize;
      } else {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyAction, kActionBlock,
                           kActionPenalize);
      }
    }
    if (HasKey(rule, kKeyPenaltyFraction)) {
      if (!rule[kKeyPenaltyFraction.data()].IsNumber()) {
        return std::format("{} must be a number >= 0", kKeyPenaltyFraction);
      }
      const double fraction = rule[kKeyPenaltyFraction.data()].GetDouble();
      // Reject negatives and non-finite values: a negative penalty would break the
      // search heuristic's admissibility. No upper bound -- a large fraction is a
      // legitimate "almost block, but keep the graph connected".
      if (!std::isfinite(fraction) || fraction < 0.0) {
        return std::format("{} must be a finite number >= 0", kKeyPenaltyFraction);
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
    auto ids = ParseIdList(args, kKeyIds);
    if (!ids) {
      return {JsonError(std::format("{} (array of at most {} strings) is required", kKeyIds,
                                    kMaxIdListSize)),
              kStatusBadRequest};
    }
    return fn(db, *ids, OutputFormat::kJson);
  };
}

// The find_routes handler: parse the args object into a RouteRequest (with the
// server-side validation the typed entry does not redo), then delegate.
HandlerResult FindRoutesHandler(const rapidjson::Value& args, const NavDatabase& db) {
  bf::RouteRequest request;
  if (!HasKey(args, kKeyDeparture) || !args[kKeyDeparture.data()].IsString() ||
      !HasKey(args, kKeyArrival) || !args[kKeyArrival.data()].IsString()) {
    return {JsonError(std::format("{} and {} are required", kKeyDeparture, kKeyArrival)),
            kStatusBadRequest};
  }
  request.departure = args[kKeyDeparture.data()].GetString();
  request.arrival = args[kKeyArrival.data()].GetString();
  // Altitude is an inclusive flight-level range. min_fl/max_fl may be given
  // together for a band, or either alone (the other defaults to it) for a
  // single level. Absent => no altitude/MORA filtering. Present-but-wrong-type
  // is a 400 (not a silent default) so clients cannot think a constraint stuck.
  {
    const bool has_min_key = HasKey(args, kKeyMinFl);
    const bool has_max_key = HasKey(args, kKeyMaxFl);
    if (has_min_key && !args[kKeyMinFl.data()].IsInt()) {
      return {JsonError(std::format("{} must be an integer", kKeyMinFl)), kStatusBadRequest};
    }
    if (has_max_key && !args[kKeyMaxFl.data()].IsInt()) {
      return {JsonError(std::format("{} must be an integer", kKeyMaxFl)), kStatusBadRequest};
    }
    if (has_min_key || has_max_key) {
      const int min_fl =
          has_min_key ? args[kKeyMinFl.data()].GetInt() : args[kKeyMaxFl.data()].GetInt();
      const int max_fl =
          has_max_key ? args[kKeyMaxFl.data()].GetInt() : args[kKeyMinFl.data()].GetInt();
      if (min_fl > max_fl) {
        return {JsonError(std::format("{} must not exceed {}", kKeyMinFl, kKeyMaxFl)),
                kStatusBadRequest};
      }
      // Flight levels are non-negative (hundreds of feet). A negative bound is
      // physically meaningless and would make the altitude-band and MORA
      // constraints reject every edge with recorded data, silently yielding
      // "no route" rather than an error. Reject it up front. (min_fl <= max_fl
      // is already enforced, so this bounds both.)
      if (min_fl < 0) {
        return {JsonError(std::format("{} and {} must be non-negative", kKeyMinFl, kKeyMaxFl)),
                kStatusBadRequest};
      }
      // Bound the top of the range: an absurdly high FL is physically
      // meaningless and only invites overflow-adjacent inputs. min_fl <= max_fl
      // is already enforced, so capping max_fl bounds both.
      if (max_fl > kMaxFl) {
        return {
            JsonError(std::format("{} and {} must not exceed {}", kKeyMinFl, kKeyMaxFl, kMaxFl)),
            kStatusBadRequest};
      }
      request.altitude = bf::FlRange{min_fl, max_fl};
    }
  }
  if (HasKey(args, kKeyLevel)) {
    if (!args[kKeyLevel.data()].IsString()) {
      return {JsonError(std::format("{} must be a string", kKeyLevel)), kStatusBadRequest};
    }
    const std::string_view level = args[kKeyLevel.data()].GetString();
    if (level == kLevelLow) {
      request.level = bf::LevelPreference::kLow;
    } else if (level == kLevelHigh) {
      request.level = bf::LevelPreference::kHigh;
    } else if (level == kLevelNone || level.empty()) {
      request.level = bf::LevelPreference::kNone;
    } else {
      return {JsonError(std::format("{} must be one of '{}', '{}', or '{}'", kKeyLevel, kLevelLow,
                                    kLevelHigh, kLevelNone)),
              kStatusBadRequest};
    }
  }
  if (HasKey(args, kKeyK)) {
    // A present-but-non-integer k (e.g. 3.5, or the string "3") must be rejected,
    // not silently dropped to the default: the caller clearly meant to set k, and
    // ignoring it would hand back one route where several were asked for.
    if (!args[kKeyK.data()].IsInt()) {
      return {JsonError(std::format("{} must be an integer", kKeyK)), kStatusBadRequest};
    }
    request.k = args[kKeyK.data()].GetInt();
  }
  // The schema declares minimum:1, but enforce it server-side too: Yen K-shortest
  // is undefined for k <= 0, and a non-positive value must not reach FindRoutes.
  if (request.k < 1) {
    return {JsonError(std::format("{} must be a positive integer (>= 1)", kKeyK)),
            kStatusBadRequest};
  }
  // Cap k as well: each extra path costs a full spur A* search, so an unbounded k
  // (e.g. 2e9) would pin a worker for a long time and let a few requests exhaust
  // the threadpool. kMaxK far exceeds any real "give me alternatives" use. (The
  // CLI does not enforce this cap -- it is a server-protection concern -- so the
  // cap lives here in the adapter, not in the typed entry.)
  if (request.k > kMaxK) {
    return {JsonError(std::format("{} must not exceed {}", kKeyK, kMaxK)), kStatusBadRequest};
  }
  auto require_string = [&](std::string_view key,
                            std::string& out) -> std::optional<HandlerResult> {
    if (!HasKey(args, key)) {
      return std::nullopt;
    }
    if (!args[key.data()].IsString()) {
      return HandlerResult{JsonError(std::format("{} must be a string", key)), kStatusBadRequest};
    }
    out = args[key.data()].GetString();
    return std::nullopt;
  };
  if (auto err = require_string(kKeyDepartureRunway, request.departure_runway)) {
    return *err;
  }
  if (auto err = require_string(kKeyArrivalRunway, request.arrival_runway)) {
    return *err;
  }
  if (auto err = require_string(kKeyDepartureSid, request.departure_sid)) {
    return *err;
  }
  if (auto err = require_string(kKeyArrivalStar, request.arrival_star)) {
    return *err;
  }
  // For the optional ID lists, tell "absent" (skip) from "present but bad"
  // (reject): a present list that is malformed or over the kMaxIdListSize cap is
  // a 400, not a silent no-op that would drop a user's avoid/force intent.
  if (HasKey(args, kKeyAvoidWaypoints)) {
    auto v = ParseIdList(args, kKeyAvoidWaypoints);
    if (!v) {
      return {JsonError(std::format("{} must be an array of at most {} strings", kKeyAvoidWaypoints,
                                    kMaxIdListSize)),
              kStatusBadRequest};
    }
    request.avoid_waypoints = std::move(*v);
  }
  if (std::optional<std::string> err = ParseAirwayRules(args, request.airway_rules)) {
    return {JsonError(*err), kStatusBadRequest};
  }
  if (HasKey(args, kKeyRandomSeed)) {
    if (!args[kKeyRandomSeed.data()].IsUint()) {
      return {JsonError(std::format("{} must be an unsigned integer", kKeyRandomSeed)),
              kStatusBadRequest};
    }
    request.random_seed = args[kKeyRandomSeed.data()].GetUint();
  }
  if (HasKey(args, kKeyForcedPoints)) {
    auto v = ParseIdList(args, kKeyForcedPoints);
    if (!v) {
      return {JsonError(std::format("{} must be an array of at most {} strings", kKeyForcedPoints,
                                    kMaxIdListSize)),
              kStatusBadRequest};
    }
    request.forced_points = std::move(*v);
  }

  return FindRoutes(db, request, OutputFormat::kJson);
}

// The lookup_procedure_legs handler: per-leg detail of one named procedure.
HandlerResult LookupProcedureLegsHandler(const rapidjson::Value& args, const NavDatabase& db) {
  if (!HasKey(args, kKeyAirport) || !args[kKeyAirport.data()].IsString() ||
      !HasKey(args, kKeyProcedure) || !args[kKeyProcedure.data()].IsString()) {
    return {JsonError(std::format("{} and {} are required", kKeyAirport, kKeyProcedure)),
            kStatusBadRequest};
  }
  return LookupProcedureLegs(db, args[kKeyAirport.data()].GetString(),
                             args[kKeyProcedure.data()].GetString(), OutputFormat::kJson);
}

// The parse_route handler: validate & expand a filed route string.
HandlerResult ParseRouteHandler(const rapidjson::Value& args, const NavDatabase& db) {
  if (!HasKey(args, kKeyRoute) || !args[kKeyRoute.data()].IsString()) {
    return {JsonError(std::format("{} (string) is required", kKeyRoute)), kStatusBadRequest};
  }
  return ParseRoute(db, args[kKeyRoute.data()].GetString(), OutputFormat::kJson);
}

}  // namespace

std::vector<NamedHandler> MakeHandlers() {
  // Built once for the caller to own. The order is the display order the MCP
  // transport presents (find_routes, parse_route, then the batch lookups); the
  // HTTP transport keys off the names, not the order.
  std::vector<NamedHandler> handlers;
  handlers.reserve(9);

  handlers.push_back({std::string(kHandlerFindRoutes), FindRoutesHandler});
  handlers.push_back({std::string(kHandlerParseRoute), ParseRouteHandler});
  handlers.push_back({std::string(kHandlerLookupWaypoints), MakeLookupAdapter(LookupWaypoints)});
  handlers.push_back({std::string(kHandlerLookupAirports), MakeLookupAdapter(LookupAirports)});
  handlers.push_back({std::string(kHandlerLookupProcedures), MakeLookupAdapter(LookupProcedures)});
  handlers.push_back({std::string(kHandlerLookupProcedureLegs), LookupProcedureLegsHandler});
  handlers.push_back({std::string(kHandlerLookupAirways), MakeLookupAdapter(LookupAirways)});
  handlers.push_back(
      {std::string(kHandlerLookupNavaidDetail), MakeLookupAdapter(LookupNavaidDetails)});
  handlers.push_back({std::string(kHandlerLookupHolds), MakeLookupAdapter(LookupHolds)});

  return handlers;
}

}  // namespace bf::service
