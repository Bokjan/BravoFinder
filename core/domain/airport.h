#pragma once

#include <string>

#include "core/domain/coordinate.h"

namespace bf {

// An airport, identified by ICAO code with its reference position. In M1 the
// airport connects to the route network via a direct (DCT) leg to the nearest
// waypoints; procedure-based connection (SID/STAR) arrives in a later milestone.
struct Airport {
  std::string icao;    // e.g. "KJFK"
  std::string region;  // two-letter ICAO region code, e.g. "K6"
  Coordinate coord;
  int elevation_ft = 0;
};

}  // namespace bf
