// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <vector>

#include "core/domain/airport.h"
#include "core/domain/airway.h"
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
// waypoint indices when the graph is built.
struct AirwayConnection {
  Ident from;
  Ident to;
  AirwaySegment segment;
};

// The raw navigation dataset produced by a loader: the inputs from which the
// route graph is built. It owns no graph itself and performs no I/O.
struct NavData {
  // AIRAC cycle parsed from the data-file header line, e.g. cycle 2601. Zero if
  // not found. Carried into the .bfdb container header.
  uint32_t cycle = 0;

  std::vector<Waypoint> waypoints;
  std::vector<NavaidDetail> navaid_details;  // parallel to routable navaids in waypoints
  std::vector<HoldFix> hold_fixes;           // from earth_hold.dat
  std::vector<AirwayConnection> airways;
  std::vector<Airport> airports;
  MoraGrid mora;
  std::vector<MsaSector> msa;  // terminal-area minimum sector altitudes
};

}  // namespace bf
