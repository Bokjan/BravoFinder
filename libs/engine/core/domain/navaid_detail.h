// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "core/domain/ident.h"
#include "core/domain/waypoint.h"

namespace bf {

// Radio navaid attributes that are not needed for routing but are useful for
// display and lookup queries. Loaded from earth_nav.dat alongside Waypoint.
struct NavaidDetail {
  Ident ident;
  WaypointKind kind = WaypointKind::kOther;
  int elev_ft = 0;
  int freq_raw = 0;  // raw dat value: NDB = kHz; VOR/ILS/DME = MHz * 100
  double range_nm = 0.0;
  double heading = 0.0;  // VOR: slaved variation; ILS/LOC: localizer bearing; DME: bias
};

}  // namespace bf
