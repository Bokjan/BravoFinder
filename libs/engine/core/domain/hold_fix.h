// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>

#include "core/domain/ident.h"

namespace bf {

// A holding pattern entry from earth_hold.dat. Covers both enroute holds
// (airport_icao == "ENRT") and terminal holds (airport_icao is the ICAO code
// of the associated airport).
struct HoldFix {
  Ident fix;
  std::string airport_icao;  // "ENRT" for enroute holds
  double inbound_course = 0.0;
  double leg_time_min = 0.0;  // outbound leg time (minutes); 0 = use leg_dist_nm
  double leg_dist_nm = 0.0;   // outbound leg distance (nm); 0 = use leg_time_min
  char turn_dir = 'R';        // 'R' = right turns, 'L' = left turns
  int min_alt_ft = 0;
  int max_alt_ft = 0;      // 0 = no upper limit
  int speed_limit_kt = 0;  // 0 = no limit
};

}  // namespace bf
