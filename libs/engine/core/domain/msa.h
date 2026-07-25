// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/domain/ident.h"

namespace bf {

// One sector of a Minimum Sector Altitude: a bearing range (from a center fix)
// within which a single minimum safe altitude applies. Bearings are magnetic
// degrees; the sector spans from `bearing_from` clockwise to the next sector's
// start. Altitude is in hundreds of feet (flight-level units) as stored in
// earth_msa.dat.
struct MsaArc {
  int bearing_from = 0;  // sector start bearing, degrees magnetic
  int alt_100ft = 0;     // minimum safe altitude, hundreds of feet
  int radius_nm = 0;     // sector radius, nautical miles
};

// A Minimum Sector Altitude record: terminal-area minimum safe altitudes
// defined as sectors around a center fix, for one airport. Loaded from
// earth_msa.dat and indexed by airport ICAO.
struct MsaSector {
  Ident center;  // the fix the sectors are measured from
  std::string airport_icao{};
  std::vector<MsaArc> arcs;
};

}  // namespace bf
