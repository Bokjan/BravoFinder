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
// the procedure heading at the fix (for the turn-angle constraint), and every
// procedure that uses this same fix. Routing searches one seeded endpoint per
// Connection; the refs let the result list all equivalent SID/STAR(+runway)
// choices without re-searching.
struct Connection {
  int fix_vertex = -1;
  double seed_distance_nm = 0.0;
  // Procedure heading at the fix in degrees [0, 360), or -1 when unknown.
  // For a SID this is the INBOUND heading (direction the procedure arrives at
  // the fix from the runway side); for a STAR the OUTBOUND heading (direction
  // it leaves the fix toward the runway). Drives the turn-angle penalty at the
  // SID-exit / STAR-entry handoff.
  double bearing = -1.0;
  std::vector<ProcedureRef> procedures;
};

// Derives network connections for an airport from its parsed CIFP procedures.
// SID connections are departure side (fly out to the fix); STAR connections are
// arrival side (fly in from the fix).
//
// A procedure hands off to the enroute network at its PUBLISHED handoff point,
// not at any fix it happens to pass:
//   - a STAR is entered at its Initial Fix (the leg whose path terminator is
//     IF), which every transition of a published STAR carries;
//   - a SID is left at its last fix-bearing leg -- the bold transition-end fix
//     on a chart. A SID's own IF legs mark where a TRANSITION begins (the fix a
//     branch forks from), which is the wrong end of the record, so the exit must
//     be derived rather than read from a path terminator.
// Restricting the handoff this way is what keeps an enroute airway from joining
// a procedure a mile off the threshold and reducing it to a zero-length stub
// (the KJFK->YSSY "...B450 TESAT STAR YSSY", arr 0.3 NM case).
//
// The handoff point must additionally be ON the enroute network, tested by
// direction (see WalkDir in the .cc): being published and being wired in are
// independent conditions. An airport whose procedures expose no on-network
// handoff point on either falls back to the fixes they pass, so procedures whose
// gates are not wired in do not strand the search.
//
// When an airport has no usable procedures at all, BuildDctFallback synthesizes
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
