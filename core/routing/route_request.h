#pragma once

#include <optional>
#include <string>

namespace bf {

// Preferred airway level when both high and low options exist.
enum class LevelPreference {
  kNone,  // no preference (default)
  kLow,   // prefer Victor (low) airways
  kHigh   // prefer Jet (high) airways
};

// A route query: departure and arrival endpoints, each an airport ICAO code or
// a waypoint ident, plus optional altitude/level preferences that drive the
// constraint layer.
struct RouteRequest {
  std::string departure;
  std::string arrival;

  // Cruise altitude as a flight level (hundreds of feet), e.g. 350 for FL350.
  // When unset, altitude-based constraints (band, MORA) are not applied, so
  // behavior matches the unconstrained shortest path.
  std::optional<int> cruise_fl;

  LevelPreference level = LevelPreference::kNone;

  // Number of candidate routes to return (Yen K-shortest). Defaults to 1.
  int k = 1;

  // Optional runway filters for procedure selection. When set, only SIDs/STARs
  // serving that runway (or runway-independent ones) are considered for the
  // departure/arrival airport. Empty means "any runway".
  std::string departure_runway;
  std::string arrival_runway;

  // Optional SID/STAR selection by name. When set, only the named procedure is
  // used to connect the departure/arrival airport; a bare name ("DEEZZ5")
  // matches any transition, and "NAME.TRANSITION" ("DEEZZ5.TOWIN") pins the
  // transition. Empty means "choose automatically". Composes with the runway
  // filters. If the airport publishes no matching procedure, FindRoutes returns
  // an Error rather than silently falling back.
  std::string departure_sid;
  std::string arrival_star;
};

}  // namespace bf
