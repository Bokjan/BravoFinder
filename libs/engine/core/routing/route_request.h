// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bf {

// Soft penalty fraction (of an edge's own length) used as the default for
// LevelPreferenceConstraint and AirwayRule::penalty_fraction. Enough to push
// matched / non-preferred edges out of the optimal route while leaving them
// usable when no alternative exists.
inline constexpr double kDefaultPenaltyFraction = 0.5;

// Preferred airway level when both high and low options exist.
enum class LevelPreference {
  kNone,  // no preference (default)
  kLow,   // prefer Victor (low) airways
  kHigh   // prefer Jet (high) airways
};

// A cruise flight-level range (hundreds of feet), inclusive on both ends. A
// single desired level is expressed as min_fl == max_fl. Drives the altitude
// band and MORA constraints: an airway segment is usable if its own [base_fl,
// top_fl] band overlaps this range, and a cell's MORA floor is cleared if the
// range's top is at or above it.
struct FlRange {
  int min_fl = 0;  // lower bound, inclusive
  int max_fl = 0;  // upper bound, inclusive
};

// One rule restricting airways by ICAO region and designator. Airway usage
// conventions are regional -- a "J" route is a terminal transition in China but a
// legal Jet route in the US -- and the source data carries no type field to tell
// them apart, so a rule is expressed as a region set plus a designator set.
//
// Matching is per-LEG, not per-airway-name. A stored designator is NOT unique to
// one physical airway: 29.6% of the names in AIRAC 2601 are reused by several
// disjoint instances (up to 18 for one name), so blocking every leg that shares a
// name would kill same-named airways worldwide -- a "region Z, prefix J" rule
// would also forbid the legal US Jet routes (128 of the 153 legs it matched were
// outside China). Matching each leg by its own endpoints' regions confines the
// rule to the airspace the user named.
struct AirwayRule {
  enum class Action : uint8_t {
    kPenalize,  // soft: add a distance-proportional cost, keeping the leg usable
    kBlock      // hard: the leg cannot be used at all
  };

  // How `designators` are compared. Both modes are needed: a category rule ("all
  // J routes") must be a prefix, while a single-airway rule must be exact --
  // 1371 designators in AIRAC 2601 are a strict prefix of another one, so a
  // prefix "J60" would also catch J603/J604/J605, and "A3" would catch 52 names.
  enum class Match : uint8_t { kExact, kPrefix };

  // ICAO region codes to match, each as a PREFIX: "Z" matches ZB/ZG/.../ZM/ZK,
  // "ZB" matches ZB only. An empty vector -- or any empty entry -- matches every
  // region. Listing several entries names a region SET in one rule: mainland
  // China is the ten FIRs ZB/ZG/ZH/ZJ/ZL/ZP/ZS/ZU/ZW/ZY, whereas the prefix "Z"
  // would also cover ZM (Mongolia) and ZK (North Korea). Free at runtime, since
  // every entry of one rule shares that rule's bit in the resolved mask.
  //
  // There is deliberately no Match mode here, unlike designators: region codes
  // are at most two characters, so a two-character entry is already exact (no
  // longer code exists to extend it). The lone corner case is the single-char
  // region "P", where a prefix also covers PA/PB/...; enumerate two-character
  // regions when that matters.
  //
  // These are ARINC 424 ICAO Codes (the two-letter region indicator), NOT airport
  // ICAO identifiers; see FixedIdent.
  std::vector<std::string> region_prefixes;

  // Airway designators to match, compared according to `match`. An empty vector
  // matches every designator. A list shares one rule bit, so "block J60, A3 and
  // W19 exactly" is ONE rule rather than three -- which also keeps a long
  // avoid-style list from exhausting AirwayRuleConstraint::kMaxRules. A
  // concurrency name ("A14-M1") is split into its designators first, so the rule
  // hits when ANY of them matches.
  std::vector<std::string> designators;

  Match match = Match::kPrefix;
  Action action = Action::kPenalize;

  // Soft penalty as a fraction of the leg's own length, used only when action is
  // kPenalize. Proportional rather than a fixed amount so short and long legs are
  // treated alike (a fixed 10 NM is punitive on a 30 NM leg and negligible on a
  // 300 NM one). Defaults to kDefaultPenaltyFraction (shared with
  // LevelPreferenceConstraint). Must be >= 0: a negative penalty would break the
  // heuristic's admissibility.
  double penalty_fraction = kDefaultPenaltyFraction;
};

// A route query: departure and arrival endpoints must be airport ICAO codes
// (waypoint endpoints are not supported — idents are not globally unique).
// Optional altitude/level preferences drive the constraint layer.
struct RouteRequest {
  std::string departure{};
  std::string arrival{};

  // Cruise altitude as an inclusive flight-level range, e.g. {350, 350} for a
  // single FL350 or {300, 400} for "anywhere FL300-FL400". When unset,
  // altitude-based constraints (band, MORA) are not applied, so behavior
  // matches the unconstrained shortest path.
  std::optional<FlRange> altitude;

  LevelPreference level = LevelPreference::kNone;

  // Number of candidate routes to return (Yen K-shortest). Defaults to 1.
  int k = 1;

  // Optional runway filters for procedure selection. When set, only SIDs/STARs
  // serving that runway (or runway-independent ones) are considered for the
  // departure/arrival airport. Empty means "any runway".
  std::string departure_runway{};
  std::string arrival_runway{};

  // Optional SID/STAR selection by name. When set, only the named procedure is
  // used to connect the departure/arrival airport; a bare name ("DEEZZ5")
  // matches any transition, and "NAME.TRANSITION" ("DEEZZ5.TOWIN") pins the
  // transition. Empty means "choose automatically". Composes with the runway
  // filters. If the airport publishes no matching procedure, FindRoutes returns
  // an Error rather than silently falling back.
  std::string departure_sid{};
  std::string arrival_star{};

  // Waypoints the route must not pass through, as an ident ("BOTON") or a full
  // "IDENT/REGION" key ("BOTON/LF"). A bare ident avoids every region's match,
  // since idents are not globally unique -- "avoid X" means avoid all X.
  std::vector<std::string> avoid_waypoints;

  // Region + designator airway rules (see AirwayRule). Empty -- the default --
  // leaves routing unchanged. When several rules match one leg, any kBlock rule
  // blocks it; otherwise the matching kPenalize rules' fractions sum. Capped at
  // AirwayRuleConstraint::kMaxRules; FindRoutes returns an Error above that rather
  // than silently dropping rules. The cap counts RULES, not regions or
  // designators: one rule may enumerate any number of both.
  //
  // This subsumes the former `avoid_airways` field (removed in 3.23.0): "avoid
  // J60" is now a rule with designators {"J60"}, match kExact and action kBlock,
  // which additionally supports confining the ban to given regions.
  std::vector<AirwayRule> airway_rules;

  // When set, perturbs edge costs by a small deterministic amount seeded by this
  // value ("random routing"): the same seed reproduces the same route, while
  // different seeds explore alternative but still valid routes. Unset => no
  // perturbation (the plain optimal route).
  std::optional<uint32_t> random_seed;

  // Ordered waypoints the route must pass through ("via" / forced points), each
  // an ident ("PSB") or a full "IDENT/REGION" key ("PSB/K6"). The search is run
  // in segments (departure -> F1 -> ... -> Fn -> arrival) and stitched, so each
  // forced point appears in order. A bare ident with several regional matches
  // resolves to the one adding the least detour (see FindRoutes). When
  // turn_penalty is enabled, each stitched candidate is re-costed across the
  // whole path so turn cost at via seams is included (per-hop costs alone would
  // miss it).
  std::vector<std::string> forced_points;
};

}  // namespace bf
