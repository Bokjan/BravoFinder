#pragma once

#include <vector>

#include "core/domain/airport.h"
#include "core/domain/airway.h"
#include "core/domain/ident.h"
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
  std::vector<Waypoint> waypoints;
  std::vector<AirwayConnection> airways;
  std::vector<Airport> airports;
};

}  // namespace bf
