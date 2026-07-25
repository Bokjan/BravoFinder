// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/domain/procedure.h"
#include "core/graph/astar.h"

namespace bf {

class GraphBuilder;

// One way a procedure references the route network: the name of the procedure
// and the transition (runway or enroute fix) that share a connection fix.
struct ProcedureRef {
  ProcedureType type = ProcedureType::kSid;
  std::string name;        // e.g. "DEEZZ5"
  std::string transition;  // enroute transition ident, runway, or "ALL"
  std::string runway;      // runway ident if known, else empty
};

// A point where procedures hand off to / pick up from the enroute network: the
// connection-fix vertex, the estimated procedure distance flown to reach it
// (from the runway, for departures) or from it (to the runway, for arrivals),
// and every procedure that uses this same fix. Routing searches one seeded
// endpoint per Connection; the refs let the result list all equivalent
// SID/STAR(+runway) choices without re-searching.
struct Connection {
  int fix_vertex = -1;
  double seed_distance_nm = 0.0;
  std::vector<ProcedureRef> procedures;
};

// Derives network connections for an airport from its parsed CIFP procedures.
// SID connections are departure side (fly out to the fix); STAR connections are
// arrival side (fly in from the fix). Only fixes that are actually on the
// enroute network are used, so procedures whose fixes are not yet wired in do
// not strand the search.
//
// When an airport has no usable procedures, BuildDctFallback synthesizes
// connections to the nearest on-network waypoints, preserving M1 coverage.
class ProcedureConnector {
 public:
  // Departure connections (from SIDs) for an airport at `airport_coord`.
  static std::vector<Connection> BuildDeparture(const CifpData& cifp,
                                                const Coordinate& airport_coord,
                                                const GraphBuilder& builder,
                                                const std::string& runway_filter);

  // Arrival connections (from STARs) for an airport at `airport_coord`.
  static std::vector<Connection> BuildArrival(const CifpData& cifp, const Coordinate& airport_coord,
                                              const GraphBuilder& builder,
                                              const std::string& runway_filter);

  // DCT fallback: connect to the nearest on-network waypoints by great-circle
  // distance. Used when no procedure connections are available. `arrival` picks
  // the direction: departures seed on fixes with an outbound airway edge (leave
  // along one), arrivals on fixes with an inbound edge (are reached along one).
  static std::vector<Connection> BuildDctFallback(const Coordinate& airport_coord,
                                                  const GraphBuilder& builder, int count,
                                                  bool arrival);

  // Convert connections to seeded A* endpoints (vertex + seed cost).
  static std::vector<SeededEndpoint> ToEndpoints(const std::vector<Connection>& connections);
};

}  // namespace bf
