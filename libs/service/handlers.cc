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

// NUL-safe string extraction: RapidJSON strings may embed '\0', so never build a
// std::string / string_view from GetString() alone (that truncates at the first
// NUL). Always pair GetString() with GetStringLength().
std::string JsonString(const rapidjson::Value& v) {
  return std::string(v.GetString(), v.GetStringLength());
}

std::string_view JsonStringView(const rapidjson::Value& v) {
  return std::string_view(v.GetString(), v.GetStringLength());
}

// Look up a member by string_view without assuming key.data() is NUL-terminated.
// Returns nullptr when absent.
const rapidjson::Value* Member(const rapidjson::Value& o, std::string_view key) {
  const auto it =
      o.FindMember(rapidjson::StringRef(key.data(), static_cast<rapidjson::SizeType>(key.size())));
  if (it == o.MemberEnd()) {
    return nullptr;
  }
  return &it->value;
}

bool HasKey(const rapidjson::Value& o, std::string_view key) { return Member(o, key) != nullptr; }

// Wrap a handler so non-object args become a 400 before any field parsing.
QueryHandler RequireObjectArgs(QueryHandler inner) {
  return [inner = std::move(inner)](const rapidjson::Value& args,
                                    const NavDatabase& db) -> HandlerResult {
    if (!args.IsObject()) {
      return {JsonError("arguments must be a JSON object"), kStatusBadRequest};
    }
    return inner(args, db);
  };
}

// Parse a string-array argument. Returns nullopt if the member is absent OR
// malformed (not an array, a non-string element, more than kMaxIdListSize
// entries, or an entry longer than kMaxIdentStringLen). Callers that need to
// tell "absent" (skip) from "present but bad" (reject) check HasKey themselves
// before deciding.
std::optional<std::vector<std::string>> ParseIdList(const rapidjson::Value& args,
                                                    std::string_view key) {
  const rapidjson::Value* arr_v = Member(args, key);
  if (!arr_v || !arr_v->IsArray()) {
    return std::nullopt;
  }
  const rapidjson::Value& arr = *arr_v;
  if (arr.Size() > kMaxIdListSize) {
    return std::nullopt;  // over the cap: treat as malformed so callers reject it
  }
  std::vector<std::string> ids;
  ids.reserve(arr.Size());
  for (const rapidjson::Value& v : arr.GetArray()) {
    if (!v.IsString()) {
      return std::nullopt;
    }
    std::string s = JsonString(v);
    if (s.size() > kMaxIdentStringLen) {
      return std::nullopt;  // over the per-id cap: treat as malformed so callers reject it
    }
    ids.push_back(std::move(s));
  }
  return ids;
}

// Parse one required string-array member of a rule object. Both region_prefixes
// and designators are required on every rule (align with the CLI, where both
// sides of '<regions>:<designators>' are mandatory). An empty array is rejected
// — use explicit ["*"] for "any", matching the CLI's bare '*'. A lone ["*"]
// clears `out` (match everything). "*" mixed with other entries, empty-string
// elements, wrong type, or oversize lists are errors. Entries are upper-cased
// (nav data is upper-case; see ToUpper).
std::optional<std::string> ParseRuleStringList(const rapidjson::Value& rule, std::string_view key,
                                               std::vector<std::string>& out) {
  const rapidjson::Value* arr_v = Member(rule, key);
  if (!arr_v) {
    return std::format("{} is required (use [\"*\"] for any)", key);
  }
  if (!arr_v->IsArray() || arr_v->Size() > kMaxIdListSize) {
    return std::format("{} must be an array of at most {} strings", key, kMaxIdListSize);
  }
  if (arr_v->Size() == 0) {
    return std::format("{} must not be empty (use [\"*\"] for any)", key);
  }
  std::vector<std::string> items;
  items.reserve(arr_v->Size());
  bool saw_star = false;
  for (const rapidjson::Value& v : arr_v->GetArray()) {
    if (!v.IsString()) {
      return std::format("{} must be an array of at most {} strings", key, kMaxIdListSize);
    }
    std::string s = JsonString(v);
    if (s.empty()) {
      return std::format("{} must not contain empty strings", key);
    }
    if (s.size() > kMaxIdentStringLen) {
      return std::format("{} entries must be at most {} characters", key, kMaxIdentStringLen);
    }
    if (s == "*") {
      saw_star = true;
    }
    items.push_back(ToUpper(std::move(s)));
  }
  if (saw_star) {
    if (items.size() != 1) {
      return std::format("{}: \"*\" cannot be mixed with other entries; use [\"*\"] alone for any",
                         key);
    }
    out.clear();
    return std::nullopt;
  }
  out = std::move(items);
  return std::nullopt;
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
  const rapidjson::Value* arr_v = Member(args, kKeyAirwayRules);
  if (!arr_v) {
    return std::nullopt;
  }
  if (!arr_v->IsArray()) {
    return std::format("{} must be an array of rule objects", kKeyAirwayRules);
  }
  const rapidjson::Value& arr = *arr_v;
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
    if (std::optional<std::string> err =
            ParseRuleStringList(rule, kKeyRegionPrefixes, parsed.region_prefixes)) {
      return err;
    }
    if (std::optional<std::string> err =
            ParseRuleStringList(rule, kKeyDesignators, parsed.designators)) {
      return err;
    }
    if (const rapidjson::Value* match_v = Member(rule, kKeyMatch)) {
      if (!match_v->IsString()) {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyMatch, kMatchExact, kMatchPrefix);
      }
      const std::string_view match = JsonStringView(*match_v);
      if (match == kMatchExact) {
        parsed.match = bf::AirwayRule::Match::kExact;
      } else if (match == kMatchPrefix) {
        parsed.match = bf::AirwayRule::Match::kPrefix;
      } else {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyMatch, kMatchExact, kMatchPrefix);
      }
    }
    if (const rapidjson::Value* action_v = Member(rule, kKeyAction)) {
      if (!action_v->IsString()) {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyAction, kActionBlock,
                           kActionPenalize);
      }
      const std::string_view action = JsonStringView(*action_v);
      if (action == kActionBlock) {
        parsed.action = bf::AirwayRule::Action::kBlock;
      } else if (action == kActionPenalize) {
        parsed.action = bf::AirwayRule::Action::kPenalize;
      } else {
        return std::format("{} must be \"{}\" or \"{}\"", kKeyAction, kActionBlock,
                           kActionPenalize);
      }
    }
    if (const rapidjson::Value* frac_v = Member(rule, kKeyPenaltyFraction)) {
      if (!frac_v->IsNumber()) {
        return std::format("{} must be a number >= 0", kKeyPenaltyFraction);
      }
      const double fraction = frac_v->GetDouble();
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

// Required string: present, non-empty, and within max_len.
std::optional<HandlerResult> RequireNonEmptyString(const rapidjson::Value& args,
                                                   std::string_view key, std::string& out,
                                                   size_t max_len) {
  const rapidjson::Value* v = Member(args, key);
  if (!v || !v->IsString()) {
    return HandlerResult{JsonError(std::format("{} (non-empty string) is required", key)),
                         kStatusBadRequest};
  }
  std::string s = JsonString(*v);
  if (s.empty()) {
    return HandlerResult{JsonError(std::format("{} must be a non-empty string", key)),
                         kStatusBadRequest};
  }
  if (s.size() > max_len) {
    return HandlerResult{JsonError(std::format("{} must be at most {} characters", key, max_len)),
                         kStatusBadRequest};
  }
  out = std::move(s);
  return std::nullopt;
}

// Optional string: absent is fine; if present it must be a non-empty string
// within max_len (an explicit empty string is a 400, not "clear to any").
std::optional<HandlerResult> OptionalNonEmptyString(const rapidjson::Value& args,
                                                    std::string_view key, std::string& out,
                                                    size_t max_len) {
  const rapidjson::Value* v = Member(args, key);
  if (!v) {
    return std::nullopt;
  }
  if (!v->IsString()) {
    return HandlerResult{JsonError(std::format("{} must be a string", key)), kStatusBadRequest};
  }
  std::string s = JsonString(*v);
  if (s.empty()) {
    return HandlerResult{JsonError(std::format("{} must be a non-empty string when present", key)),
                         kStatusBadRequest};
  }
  if (s.size() > max_len) {
    return HandlerResult{JsonError(std::format("{} must be at most {} characters", key, max_len)),
                         kStatusBadRequest};
  }
  out = std::move(s);
  return std::nullopt;
}

// Build a batch-lookup handler from a typed LookupX entry: parse the ids array
// (400 on a malformed/missing/empty list), then delegate to the entry in JSON.
template <class Fn>
QueryHandler MakeLookupAdapter(Fn fn) {
  return RequireObjectArgs([fn](const rapidjson::Value& args,
                                const NavDatabase& db) -> HandlerResult {
    auto ids = ParseIdList(args, kKeyIds);
    if (!ids) {
      return {JsonError(std::format("{} (array of at most {} strings) is required", kKeyIds,
                                    kMaxIdListSize)),
              kStatusBadRequest};
    }
    if (ids->empty()) {
      return {JsonError(std::format("{} must be a non-empty array", kKeyIds)), kStatusBadRequest};
    }
    return fn(db, *ids, OutputFormat::kJson);
  });
}

// The find_routes handler: parse the args object into a RouteRequest (with the
// server-side validation the typed entry does not redo), then delegate.
HandlerResult FindRoutesHandler(const rapidjson::Value& args, const NavDatabase& db) {
  bf::RouteRequest request;
  if (auto err =
          RequireNonEmptyString(args, kKeyDeparture, request.departure, kMaxIdentStringLen)) {
    return *err;
  }
  if (auto err = RequireNonEmptyString(args, kKeyArrival, request.arrival, kMaxIdentStringLen)) {
    return *err;
  }
  // Altitude is an inclusive flight-level range. min_fl/max_fl may be given
  // together for a band, or either alone (the other defaults to it) for a
  // single level. Absent => no altitude/MORA filtering. Present-but-wrong-type
  // is a 400 (not a silent default) so clients cannot think a constraint stuck.
  {
    const rapidjson::Value* min_v = Member(args, kKeyMinFl);
    const rapidjson::Value* max_v = Member(args, kKeyMaxFl);
    const bool has_min_key = min_v != nullptr;
    const bool has_max_key = max_v != nullptr;
    if (has_min_key && !min_v->IsInt()) {
      return {JsonError(std::format("{} must be an integer", kKeyMinFl)), kStatusBadRequest};
    }
    if (has_max_key && !max_v->IsInt()) {
      return {JsonError(std::format("{} must be an integer", kKeyMaxFl)), kStatusBadRequest};
    }
    if (has_min_key || has_max_key) {
      const int min_fl = has_min_key ? min_v->GetInt() : max_v->GetInt();
      const int max_fl = has_max_key ? max_v->GetInt() : min_v->GetInt();
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
  if (const rapidjson::Value* level_v = Member(args, kKeyLevel)) {
    if (!level_v->IsString()) {
      return {JsonError(std::format("{} must be a string", kKeyLevel)), kStatusBadRequest};
    }
    const std::string_view level = JsonStringView(*level_v);
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
  if (const rapidjson::Value* k_v = Member(args, kKeyK)) {
    // A present-but-non-integer k (e.g. 3.5, or the string "3") must be rejected,
    // not silently dropped to the default: the caller clearly meant to set k, and
    // ignoring it would hand back one route where several were asked for.
    if (!k_v->IsInt()) {
      return {JsonError(std::format("{} must be an integer", kKeyK)), kStatusBadRequest};
    }
    request.k = k_v->GetInt();
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
  if (auto err = OptionalNonEmptyString(args, kKeyDepartureRunway, request.departure_runway,
                                        kMaxIdentStringLen)) {
    return *err;
  }
  if (auto err = OptionalNonEmptyString(args, kKeyArrivalRunway, request.arrival_runway,
                                        kMaxIdentStringLen)) {
    return *err;
  }
  if (auto err = OptionalNonEmptyString(args, kKeyDepartureSid, request.departure_sid,
                                        kMaxIdentStringLen)) {
    return *err;
  }
  if (auto err =
          OptionalNonEmptyString(args, kKeyArrivalStar, request.arrival_star, kMaxIdentStringLen)) {
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
  if (const rapidjson::Value* seed_v = Member(args, kKeyRandomSeed)) {
    if (!seed_v->IsUint()) {
      return {JsonError(std::format("{} must be an unsigned integer", kKeyRandomSeed)),
              kStatusBadRequest};
    }
    request.random_seed = seed_v->GetUint();
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
  std::string airport;
  std::string procedure;
  if (auto err = RequireNonEmptyString(args, kKeyAirport, airport, kMaxIdentStringLen)) {
    return *err;
  }
  if (auto err = RequireNonEmptyString(args, kKeyProcedure, procedure, kMaxIdentStringLen)) {
    return *err;
  }
  return LookupProcedureLegs(db, airport, procedure, OutputFormat::kJson);
}

// The parse_route handler: validate & expand a filed route string.
HandlerResult ParseRouteHandler(const rapidjson::Value& args, const NavDatabase& db) {
  std::string route;
  if (auto err = RequireNonEmptyString(args, kKeyRoute, route, kMaxRouteStringLen)) {
    return *err;
  }
  return ParseRoute(db, route, OutputFormat::kJson);
}

}  // namespace

std::vector<NamedHandler> MakeHandlers() {
  // Built once for the caller to own. The order is the display order the MCP
  // transport presents (find_routes, parse_route, then the batch lookups); the
  // HTTP transport keys off the names, not the order.
  std::vector<NamedHandler> handlers;
  handlers.reserve(10);

  handlers.push_back({std::string(kHandlerFindRoutes), RequireObjectArgs(FindRoutesHandler)});
  handlers.push_back({std::string(kHandlerParseRoute), RequireObjectArgs(ParseRouteHandler)});
  handlers.push_back({std::string(kHandlerLookupWaypoints), MakeLookupAdapter(LookupWaypoints)});
  handlers.push_back({std::string(kHandlerLookupAirports), MakeLookupAdapter(LookupAirports)});
  handlers.push_back({std::string(kHandlerLookupProcedures), MakeLookupAdapter(LookupProcedures)});
  handlers.push_back(
      {std::string(kHandlerLookupProcedureLegs), RequireObjectArgs(LookupProcedureLegsHandler)});
  handlers.push_back({std::string(kHandlerLookupAirways), MakeLookupAdapter(LookupAirways)});
  handlers.push_back(
      {std::string(kHandlerLookupNavaidDetail), MakeLookupAdapter(LookupNavaidDetails)});
  handlers.push_back({std::string(kHandlerLookupHolds), MakeLookupAdapter(LookupHolds)});
  handlers.push_back({std::string(kHandlerLookupMsa), MakeLookupAdapter(LookupMsa)});

  return handlers;
}

}  // namespace bf::service
