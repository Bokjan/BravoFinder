// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>

#include "core/domain/coordinate.h"
#include "core/domain/ident.h"

namespace bf {

// The kind of a navigation point. Enroute waypoints (fixes) and radio navaids
// (VOR/DME/NDB/TACAN) are both vertices in the route graph; the kind is kept
// for display and for future constraints (e.g. navaid-only routing).
// Underlying type is uint8_t: the on-disk vertex record stores it as a U8
// (see graph_codec.cc static_assert), and the per-vertex kinds_ array is large
// enough that the default int width would waste ~0.8 MB.
enum class WaypointKind : uint8_t {
  kFix,    // enroute or terminal waypoint (earth_fix.dat)
  kVor,    // VOR / VOR-DME
  kNdb,    // NDB
  kDme,    // DME / TACAN
  kOther,  // any other navaid row kept as a routable point
};

// A point in the navigation network: an identified position with a kind.
struct Waypoint {
  Ident ident;
  Coordinate coord;
  WaypointKind kind = WaypointKind::kFix;
};

}  // namespace bf
