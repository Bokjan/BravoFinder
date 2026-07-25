// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bf {

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

// A route query: departure and arrival endpoints, each an airport ICAO code or
// a waypoint ident, plus optional altitude/level preferences that drive the
// constraint layer.
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

  // Airways the route must not traverse, by designator ("J60"). Matches against
  // each airway's designators, so an avoided "J60" also blocks concurrency
  // segments recorded as "J60-V123".
  std::vector<std::string> avoid_airways;

  // When set, perturbs edge costs by a small deterministic amount seeded by this
  // value ("random routing"): the same seed reproduces the same route, while
  // different seeds explore alternative but still valid routes. Unset => no
  // perturbation (the plain optimal route).
  std::optional<uint32_t> random_seed;

  // Ordered waypoints the route must pass through ("via" / forced points), each
  // an ident ("PSB") or a full "IDENT/REGION" key ("PSB/K6"). The search is run
  // in segments (departure -> F1 -> ... -> Fn -> arrival) and stitched, so each
  // forced point appears in order. A bare ident with several regional matches
  // resolves to the one adding the least detour (see FindRoutes).
  std::vector<std::string> forced_points;
};

}  // namespace bf
