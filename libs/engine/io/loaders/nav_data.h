// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "core/domain/airport.h"
#include "core/domain/airway.h"
#include "core/domain/coordinate.h"
#include "core/domain/hold_fix.h"
#include "core/domain/ident.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/domain/navaid_detail.h"
#include "core/domain/waypoint.h"

namespace bf {

// A directed connection between two waypoints along an airway segment, as
// loaded from source data. The graph builder turns these into edges (honoring
// AirwaySegment::direction). Endpoints are referenced by Ident and resolved to
// waypoint indices when the graph is built. An (ident, region) key may name
// several distinct points (see NavData::waypoints), so a loader that knows an
// endpoint's position passes it as a hint and the builder picks the matching
// same-key point; without a hint the first-loaded point wins.
struct AirwayConnection {
  Ident from;
  Ident to;
  AirwaySegment segment;
  std::optional<Coordinate> from_hint{};
  std::optional<Coordinate> to_hint{};
};

// The raw navigation dataset produced by a loader: the inputs from which the
// route graph is built. It owns no graph itself and performs no I/O.
struct NavData {
  // AIRAC cycle parsed from the data-file header line, e.g. cycle 2601. Zero if
  // not found. Carried into the .bfdb container header.
  uint32_t cycle = 0;

  // Routable points. An (ident, region) key is not unique: terminal fixes such
  // as DER06/LF are published per airport, and a VOR and an NDB may share an
  // ident. Loaders merge only same-key rows that are the same physical point
  // (within kSamePointNm, see point_dedup.h) and keep distinct ones, so
  // procedure legs and airways can be wired to the right one by position.
  std::vector<Waypoint> waypoints;
  std::vector<NavaidDetail> navaid_details;  // parallel to routable navaids in waypoints
  std::vector<HoldFix> hold_fixes;           // from earth_hold.dat
  std::vector<AirwayConnection> airways;
  std::vector<Airport> airports;
  MoraGrid mora;
  std::vector<MsaSector> msa;  // terminal-area minimum sector altitudes
};

}  // namespace bf
