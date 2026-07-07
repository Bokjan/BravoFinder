#pragma once

#include "core/domain/coordinate.h"
#include "core/domain/ident.h"

namespace bf {

// The kind of a navigation point. Enroute waypoints (fixes) and radio navaids
// (VOR/DME/NDB/TACAN) are both vertices in the route graph; the kind is kept
// for display and for future constraints (e.g. navaid-only routing).
enum class WaypointKind {
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
