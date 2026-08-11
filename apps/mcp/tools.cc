// SPDX-License-Identifier: MIT
// tools.cc — the MCP capabilities exposed by bf-mcp.
//
// The query logic itself lives in bf::service (libs/service/), shared with the
// HTTP transport. This file owns only the MCP-specific dressing: each tool's
// human/LLM-facing description and its JSON-Schema input descriptor. MakeTools()
// pulls the shared handlers by name, attaches that dressing, and adapts each
// handler's HandlerResult{body, status} into the {json_text, is_error} pair the
// stdio server writes (is_error = status >= 400). Adding a tool means adding a
// handler in bf::service and one metadata entry here.

#include "tools.h"

#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "api_keys.h"
#include "core/constraints/airway_rule_constraint.h"
#include "core/routing/route_request.h"
#include "handlers.h"
#include "rapidjson/document.h"

namespace bf::mcp {

namespace {

// Parse a JSON-Schema string into a Document. The Document owns its allocator,
// so it can be moved into a Tool and kept alive there (no statics, no globals).
rapidjson::Document ParseSchema(const char* json) {
  rapidjson::Document doc;
  doc.Parse(json);
  return doc;
}

// Quote a wire key for embedding in a JSON-Schema string.
std::string JKey(std::string_view key) { return "\"" + std::string(key) + "\""; }

using bf::service::kActionBlock;
using bf::service::kActionPenalize;
using bf::service::kHandlerFindRoutes;
using bf::service::kHandlerLookupAirports;
using bf::service::kHandlerLookupAirways;
using bf::service::kHandlerLookupHolds;
using bf::service::kHandlerLookupNavaidDetail;
using bf::service::kHandlerLookupProcedureLegs;
using bf::service::kHandlerLookupProcedures;
using bf::service::kHandlerLookupWaypoints;
using bf::service::kHandlerParseRoute;
using bf::service::kKeyAction;
using bf::service::kKeyAirport;
using bf::service::kKeyAirwayRules;
using bf::service::kKeyArrival;
using bf::service::kKeyArrivalRunway;
using bf::service::kKeyArrivalStar;
using bf::service::kKeyAvoidWaypoints;
using bf::service::kKeyDeparture;
using bf::service::kKeyDepartureRunway;
using bf::service::kKeyDepartureSid;
using bf::service::kKeyDesignators;
using bf::service::kKeyForcedPoints;
using bf::service::kKeyIds;
using bf::service::kKeyK;
using bf::service::kKeyLevel;
using bf::service::kKeyMatch;
using bf::service::kKeyMaxFl;
using bf::service::kKeyMinFl;
using bf::service::kKeyPenaltyFraction;
using bf::service::kKeyProcedure;
using bf::service::kKeyRandomSeed;
using bf::service::kKeyRegionPrefixes;
using bf::service::kKeyRoute;
using bf::service::kLevelHigh;
using bf::service::kLevelLow;
using bf::service::kLevelNone;
using bf::service::kMatchExact;
using bf::service::kMatchPrefix;
using bf::service::kMaxK;

// JSON-Schema for find_routes, built at runtime so maximum/maxItems and property
// names track the shared caps / api_keys.h instead of duplicating literals.
std::string FindRoutesSchema() {
  const auto max_rules = bf::AirwayRuleConstraint::kMaxRules;
  const auto max_u32 = std::numeric_limits<uint32_t>::max();
  return std::string(R"({"type":"object","properties":{)") + JKey(kKeyDeparture) +
         R"(:{"type":"string","description":"Departure airport ICAO or waypoint ident."},)" +
         JKey(kKeyArrival) +
         R"(:{"type":"string","description":"Arrival airport ICAO or waypoint ident."},)" +
         JKey(kKeyMinFl) +
         R"(:{"type":"integer","description":"Lower bound of the cruise flight-level range, in hundreds of feet (e.g. 300 for FL300). May be given alone for a single level. Setting min_fl and/or max_fl enables altitude/MORA constraint filtering."},)" +
         JKey(kKeyMaxFl) +
         R"(:{"type":"integer","description":"Upper bound of the cruise flight-level range, in hundreds of feet (e.g. 400 for FL400). May be given alone for a single level."},)" +
         JKey(kKeyLevel) + R"(:{"type":"string","enum":[)" + JKey(kLevelNone) + "," +
         JKey(kLevelLow) + "," + JKey(kLevelHigh) +
         R"(],"description":"Preferred airway level: none=no preference (default), low=prefer Victor low airways, high=prefer Jet high airways."},)" +
         JKey(kKeyK) + R"(:{"type":"integer","minimum":1,"maximum":)" + std::to_string(kMaxK) +
         R"(,"description":"Number of candidate routes to return (Yen K-shortest). Defaults to 1; capped at )" +
         std::to_string(kMaxK) + R"(."},)" + JKey(kKeyDepartureRunway) +
         R"(:{"type":"string","description":"Restrict the SID to this departure runway, e.g. RW31L. Empty=any."},)" +
         JKey(kKeyArrivalRunway) +
         R"(:{"type":"string","description":"Restrict the STAR to this arrival runway, e.g. RW25L. Empty=any."},)" +
         JKey(kKeyDepartureSid) +
         R"(:{"type":"string","description":"Pin a specific SID by name, e.g. DEEZZ5 or DEEZZ5.TOWIN. Empty=auto."},)" +
         JKey(kKeyArrivalStar) +
         R"(:{"type":"string","description":"Pin a specific STAR by name, e.g. LENDY6 or LENDY6.HAAYS. Empty=auto."},)" +
         JKey(kKeyAvoidWaypoints) +
         R"(:{"type":"array","items":{"type":"string"},"description":"Waypoints to route around, each an ident (BOTON) or IDENT/ARINC424_ICAO_CODE (BOTON/LF). A bare ident avoids all regions' matches."},)" +
         JKey(kKeyAirwayRules) + R"(:{"type":"array","maxItems":)" + std::to_string(max_rules) +
         R"(,"description":"Restrict airways by ICAO region and designator. Each rule blocks or penalizes every airway LEG whose designator matches and whose either endpoint lies in a matching region. Matching is per-leg, not per-airway-name, because a designator is not unique to one physical airway (29.6% of names are reused by disjoint instances) -- a name-level ban would forbid same-named airways worldwide. Several rules may match one leg: any block rule wins, otherwise the penalize fractions sum. Both region_prefixes and designators are required on every rule; use [\"*\"] alone for 'any' on that side (an empty array is rejected). Replaces the former avoid_airways: 'avoid J60' is {\"region_prefixes\":[\"*\"],\"designators\":[\"J60\"],\"match\":\"exact\",\"action\":\"block\"}.","items":{"type":"object","properties":{)" +
         JKey(kKeyRegionPrefixes) +
         R"(:{"type":"array","items":{"type":"string"},"description":"Required. ARINC 424 ICAO region codes (NOT airport identifiers), each matched as a prefix. Use [\"*\"] alone for any region (empty [] is rejected). Listing several names a region set in ONE rule, which is free at runtime and often necessary for precision: prefix \"Z\" covers ZM (Mongolia) and ZK (North Korea) besides China, so mainland China is the ten FIRs ZB,ZG,ZH,ZJ,ZL,ZP,ZS,ZU,ZW,ZY."},)" +
         JKey(kKeyDesignators) +
         R"(:{"type":"array","items":{"type":"string"},"description":"Required. Airway designators, compared per the match field. Use [\"*\"] alone for any designator (empty [] is rejected). Concurrency names (A14-M1) are split first, so a rule hits when any of their designators matches."},)" +
         JKey(kKeyMatch) + R"(:{"type":"string","enum":[)" + JKey(kMatchExact) + "," +
         JKey(kMatchPrefix) +
         R"(],"description":"How designators are compared; defaults to prefix. Use exact to name individual airways: 1371 designators are a strict prefix of another one, so prefix \"J60\" would also match J603/J604/J605, and \"A3\" would match 52 names. Use prefix for a category rule such as all J routes."},)" +
         JKey(kKeyAction) + R"(:{"type":"string","enum":[)" + JKey(kActionBlock) + "," +
         JKey(kActionPenalize) +
         R"(],"description":"block excludes matching legs outright; penalize (the default) adds a soft cost, keeping them usable. Prefer penalize for bulk region rules: block is an irreversible connectivity break and can leave an airport that depends on a single airway with no route at all."},)" +
         JKey(kKeyPenaltyFraction) +
         R"(:{"type":"number","minimum":0,"description":"Soft penalty as a fraction of each leg's own length, used only when action is penalize. Defaults to )" +
         std::to_string(bf::kDefaultPenaltyFraction) +
         R"(, which pushes matched airways out of the optimal route while leaving them usable when no alternative exists. Larger values steer harder; there is no upper bound."}},"required":[)" +
         JKey(kKeyRegionPrefixes) + "," + JKey(kKeyDesignators) + "]}}," + JKey(kKeyRandomSeed) +
         R"(:{"type":"integer","minimum":0,"maximum":)" + std::to_string(max_u32) +
         R"(,"description":"Seed for reproducible route diversity. The same seed always yields the same route; different seeds explore alternatives. Omit for the plain optimal route."},)" +
         JKey(kKeyForcedPoints) +
         R"(:{"type":"array","items":{"type":"string"},"description":"Ordered waypoints the route must pass through (via points), each an ident (PSB) or IDENT/ARINC424_ICAO_CODE (PSB/K6). The response echoes them resolved as IDENT/ARINC424_ICAO_CODE."}},)" +
         R"("required":[)" + JKey(kKeyDeparture) + "," + JKey(kKeyArrival) + "]}";
}

std::string IdsSchema(std::string_view description) {
  return std::string(R"({"type":"object","properties":{)") + JKey(kKeyIds) +
         R"(:{"type":"array","items":{"type":"string"},"description":")" +
         std::string(description) + R"("}},"required":[)" + JKey(kKeyIds) + "]}";
}

std::string ParseRouteSchema() {
  return std::string(R"({"type":"object","properties":{)") + JKey(kKeyRoute) +
         R"(:{"type":"string","description":"Filed route string, e.g. 'KJFK SID CANDR J60 PSB ... STAR KLAX'."}},)"
         R"("required":[)" +
         JKey(kKeyRoute) + "]}";
}

std::string ProcedureLegsSchema() {
  return std::string(R"({"type":"object","properties":{)") + JKey(kKeyAirport) +
         R"(:{"type":"string","description":"Airport ICAO code, e.g. KJFK."},)" +
         JKey(kKeyProcedure) +
         R"(:{"type":"string","description":"Published procedure name, e.g. DEEZZ5 or LENDY6."}},)"
         R"("required":[)" +
         JKey(kKeyAirport) + "," + JKey(kKeyProcedure) + "]}";
}

// The nine tools' descriptions, in MCP display order. `name` must match a
// bf::service handler name; MakeTools() pairs them up. Schemas are filled from
// SchemaFor() so property names share api_keys.h with the JSON-args adapter.
struct ToolMetaDesc {
  std::string_view name;
  const char* description;
};

const ToolMetaDesc kToolMeta[] = {
    {kHandlerFindRoutes,
     "Find up to k candidate routes between two endpoints (airport ICAO or "
     "waypoint ident), honoring airway level and cruise-altitude constraints. "
     "Returns an ICAO filed-flight-plan style route string plus per-leg detail."},
    {kHandlerParseRoute,
     "Validate and expand a filed-flight-plan route string (the reverse of "
     "find_routes). Given \"[DEP] [SID] FIX (AWY FIX | DCT FIX)* [STAR] [ARR]\", "
     "checks that each airway connects its bracketing fixes, expands airways to "
     "their intermediate points, totals the distance, and returns the resolved "
     "route. Errors name the offending token when the route is invalid."},
    {kHandlerLookupWaypoints,
     "Look up waypoints / navaids by ident. An ident is reused across regions, "
     "so each id returns a group of matches (ident, region, coordinate, kind, "
     "on-network flag). Batch: one group per id, empty when not found."},
    {kHandlerLookupAirports,
     "Look up airports by ICAO code. Returns coordinates, elevation, and "
     "whether the airport publishes terminal procedures. Batch: one result "
     "per id, null when not found."},
    {kHandlerLookupProcedures,
     "Look up published terminal procedures (SID/STAR/approach) by airport "
     "ICAO. Returns a summary per procedure (type, name, transition, runway). "
     "Batch: one result per id, null when unknown or no CIFP data."},
    {kHandlerLookupProcedureLegs,
     "Look up the per-leg detail of one named terminal procedure at an airport. "
     "Given an airport ICAO and a procedure name (e.g. DEEZZ5), returns every "
     "transition of that procedure with its ordered legs: fix, path terminator, "
     "magnetic course, distance, altitude constraint, RNP, turn direction, and "
     "speed limit. Errors when the airport is unknown or publishes no such procedure."},
    {kHandlerLookupAirways,
     "Look up airways by designator (e.g. Y28). Returns every directed "
     "segment carrying that name with distances and flight-level bounds. "
     "Batch: one result per id, null when no segment uses the name."},
    {kHandlerLookupNavaidDetail,
     "Look up detailed radio-navaid attributes by ident (frequency, service "
     "range, elevation, and heading/variation). An ident is reused across "
     "regions, so each id returns a group of matches. freq_raw is kHz for NDBs "
     "and MHz*100 for VOR/DME/ILS. Requires a detail cache; empty otherwise."},
    {kHandlerLookupHolds,
     "Look up holding patterns by fix ident (inbound course, outbound leg "
     "time/distance, turn direction, altitude window, speed limit). Each id "
     "returns a group of holds across all regions/airports at that fix. "
     "Requires a detail cache; empty otherwise."},
};

std::string SchemaFor(std::string_view name) {
  if (name == kHandlerFindRoutes) {
    return FindRoutesSchema();
  }
  if (name == kHandlerParseRoute) {
    return ParseRouteSchema();
  }
  if (name == kHandlerLookupProcedureLegs) {
    return ProcedureLegsSchema();
  }
  if (name == kHandlerLookupWaypoints) {
    return IdsSchema(
        "One or more waypoint idents to look up. Each result is a group parallel to this list.");
  }
  if (name == kHandlerLookupAirports) {
    return IdsSchema("One or more airport ICAO codes. Results are parallel to this list.");
  }
  if (name == kHandlerLookupProcedures) {
    return IdsSchema("One or more airport ICAO codes. Results are parallel to this list.");
  }
  if (name == kHandlerLookupAirways) {
    return IdsSchema("One or more airway designators. Results are parallel to this list.");
  }
  if (name == kHandlerLookupNavaidDetail) {
    return IdsSchema("One or more navaid idents. Each result is a group parallel to this list.");
  }
  if (name == kHandlerLookupHolds) {
    return IdsSchema("One or more hold fix idents. Each result is a group parallel to this list.");
  }
  return {};
}

// Adapt a shared bf::service handler into the MCP ToolHandler shape: run it and
// project its status onto is_error (status >= 400), keeping the JSON body and
// the elapsed_ms timing verbatim so MCP behavior is unchanged.
ToolHandler AdaptHandler(bf::service::QueryHandler handler) {
  return [handler = std::move(handler)](const rapidjson::Value& args,
                                        const bf::NavDatabase& db) -> ToolResult {
    bf::service::HandlerResult result = handler(args, db);
    return {std::move(result.body), result.status >= bf::service::kErrorStatusThreshold,
            result.elapsed_ms};
  };
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

std::vector<Tool> MakeTools() {
  // Pull the shared handlers and index them by name, then build a Tool per
  // metadata entry (which fixes the display order). Each meta.name matches a
  // handler name, so every tool gets its logic; a metadata entry with no handler
  // is skipped defensively (cannot happen with the stable names above).
  std::unordered_map<std::string, bf::service::QueryHandler> by_name;
  for (bf::service::NamedHandler& nh : bf::service::MakeHandlers()) {
    by_name.emplace(std::move(nh.name), std::move(nh.handler));
  }

  std::vector<Tool> tools;
  tools.reserve(std::size(kToolMeta));
  for (const ToolMetaDesc& meta : kToolMeta) {
    auto it = by_name.find(std::string(meta.name));
    // A tool's name must match a handler registered by bf::service::MakeHandlers();
    // a name drift there would otherwise silently drop the tool. Catch it in
    // debug builds.
    assert(it != by_name.end() && "MCP tool name has no matching bf::service handler");
    if (it == by_name.end()) {
      continue;
    }
    const std::string schema_owned = SchemaFor(meta.name);
    tools.emplace_back(std::string(meta.name), meta.description, ParseSchema(schema_owned.c_str()),
                       AdaptHandler(std::move(it->second)));
  }
  return tools;
}

}  // namespace bf::mcp
