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
};

}  // namespace bf
