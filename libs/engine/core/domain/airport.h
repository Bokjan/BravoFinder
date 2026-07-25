// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>

#include "core/domain/coordinate.h"

namespace bf {

// An airport, identified by ICAO code with its reference position. The airport
// joins the route network either via a direct (DCT) leg to nearby waypoints or
// via a SID/STAR procedure connection.
struct Airport {
  std::string icao;    // e.g. "KJFK"
  std::string region;  // two-letter ICAO region code, e.g. "K6"
  Coordinate coord;
  int elevation_ft = 0;
};

}  // namespace bf
