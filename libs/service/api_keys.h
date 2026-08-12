// SPDX-License-Identifier: MIT
#pragma once

#include <string_view>

namespace bf::service {

// Wire-facing names shared by the JSON-args adapter (handlers.cc), MCP tool
// metadata/schema (apps/mcp/tools.cc), and the HTTP route table
// (apps/http/router.cc). Kept as named string_views so a typo at one site
// cannot silently desync the others (e.g. schema says "forced_points" while
// the adapter reads "force_points").

// ---- Handler / tool names (MakeHandlers / MakeTools / kRoutes) ----

inline constexpr std::string_view kHandlerFindRoutes = "find_routes";
inline constexpr std::string_view kHandlerParseRoute = "parse_route";
inline constexpr std::string_view kHandlerLookupWaypoints = "lookup_waypoints";
inline constexpr std::string_view kHandlerLookupAirports = "lookup_airports";
inline constexpr std::string_view kHandlerLookupProcedures = "lookup_procedures";
inline constexpr std::string_view kHandlerLookupProcedureLegs = "lookup_procedure_legs";
inline constexpr std::string_view kHandlerLookupAirways = "lookup_airways";
inline constexpr std::string_view kHandlerLookupNavaidDetail = "lookup_navaid_detail";
inline constexpr std::string_view kHandlerLookupHolds = "lookup_holds";
inline constexpr std::string_view kHandlerLookupMsa = "lookup_msa";

// ---- Top-level request argument keys ----

inline constexpr std::string_view kKeyDeparture = "departure";
inline constexpr std::string_view kKeyArrival = "arrival";
inline constexpr std::string_view kKeyMinFl = "min_fl";
inline constexpr std::string_view kKeyMaxFl = "max_fl";
inline constexpr std::string_view kKeyLevel = "level";
inline constexpr std::string_view kKeyK = "k";
inline constexpr std::string_view kKeyDepartureRunway = "departure_runway";
inline constexpr std::string_view kKeyArrivalRunway = "arrival_runway";
inline constexpr std::string_view kKeyDepartureSid = "departure_sid";
inline constexpr std::string_view kKeyArrivalStar = "arrival_star";
inline constexpr std::string_view kKeyAvoidWaypoints = "avoid_waypoints";
inline constexpr std::string_view kKeyAirwayRules = "airway_rules";
inline constexpr std::string_view kKeyRandomSeed = "random_seed";
inline constexpr std::string_view kKeyForcedPoints = "forced_points";
inline constexpr std::string_view kKeyIds = "ids";
inline constexpr std::string_view kKeyAirport = "airport";
inline constexpr std::string_view kKeyProcedure = "procedure";
inline constexpr std::string_view kKeyRoute = "route";

// ---- airway_rules object fields ----

inline constexpr std::string_view kKeyRegionPrefixes = "region_prefixes";
inline constexpr std::string_view kKeyDesignators = "designators";
inline constexpr std::string_view kKeyMatch = "match";
inline constexpr std::string_view kKeyAction = "action";
inline constexpr std::string_view kKeyPenaltyFraction = "penalty_fraction";

// ---- Enum-like argument values ----

inline constexpr std::string_view kMatchExact = "exact";
inline constexpr std::string_view kMatchPrefix = "prefix";
inline constexpr std::string_view kActionBlock = "block";
inline constexpr std::string_view kActionPenalize = "penalize";
inline constexpr std::string_view kLevelNone = "none";
inline constexpr std::string_view kLevelLow = "low";
inline constexpr std::string_view kLevelHigh = "high";

}  // namespace bf::service
