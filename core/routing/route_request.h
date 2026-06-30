#pragma once

#include <string>

namespace bf {

// A route query: departure and arrival endpoints, each an airport ICAO code or
// a waypoint ident. Altitude/level preferences arrive with the constraint work
// in a later milestone.
struct RouteRequest {
  std::string departure;
  std::string arrival;
};

}  // namespace bf
