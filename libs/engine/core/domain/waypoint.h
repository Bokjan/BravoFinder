// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>

#include "core/domain/coordinate.h"
#include "core/domain/ident.h"

namespace bf {

// The kind of a navigation point. Enroute waypoints (fixes) and radio navaids
// (VOR/DME/NDB/TACAN) are both vertices in the route graph; the kind is kept
// for display and for future constraints (e.g. navaid-only routing). The last
// two kinds only ever appear in navaid details, never as graph vertices (see
// IsGraphVertexKind).
// Underlying type is uint8_t: the on-disk vertex record stores it as a U8
// (see graph_codec.cc static_assert), and the per-vertex kinds_ array is large
// enough that the default int width would waste ~0.8 MB.
enum class WaypointKind : uint8_t {
  kFix,     // enroute or terminal waypoint (earth_fix.dat)
  kVor,     // VOR / VOR-DME
  kNdb,     // NDB
  kDme,     // DME / TACAN
  kOther,   // a navaid of a type the loader does not recognize
  kIlsDme,  // DME paired with an ILS/LOC (or MLS) approach aid
};

// Whether a point of `kind` becomes a route-graph vertex. ILS/MLS DMEs are not:
// no airway touches them and, in cycle 2601, no X-Plane or Fenix procedure leg
// uses one as its fix, yet as vertices they would add ~3.5k ident candidates
// (French ILS idents such as RL collide with NDBs) and could merge with a
// same-key NDB in the loaders' same-point dedup. Every loader applies this
// before deduplicating, so no source puts them in the graph; ILS DMEs stay
// queryable through the navaid details.
constexpr bool IsGraphVertexKind(WaypointKind kind) {
  return kind != WaypointKind::kOther && kind != WaypointKind::kIlsDme;
}

// A point in the navigation network: an identified position with a kind.
struct Waypoint {
  Ident ident;
  Coordinate coord;
  WaypointKind kind = WaypointKind::kFix;
};

}  // namespace bf
