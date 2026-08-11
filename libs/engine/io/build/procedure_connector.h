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
  // For kApproach: the IAF ident this ref connects through (may differ from the
  // Connection's fix_vertex when a proxy on-network fix stands in for an
  // off-network IAF). Empty for SID/STAR.
  std::string iaf;
};

// A point where procedures hand off to / pick up from the enroute network: the
// connection-fix vertex, the estimated procedure distance flown to reach it
// (from the runway, for departures) or from it (to the runway, for arrivals),
// the procedure heading at the fix (for the turn-angle constraint), and every
// procedure that uses this same fix. Routing searches one seeded endpoint per
// Connection; the refs let the result list all equivalent SID/STAR(+runway)
// choices without re-searching.
//
// When a published SID/STAR gate is off-network, `fix_vertex` is a nearby
// on-network proxy F and `splice_vertex` holds the gate (ENTRY/EXIT) that
// MakeRoute must insert into the filed string as `… F DCT ENTRY STAR ARR` /
// `DEP SID EXIT DCT F …`. On-network gates and approach/DCT connections leave
// `splice_vertex == kNoVertex` (no insertion).
struct Connection {
  int fix_vertex = kNoVertex;
  double seed_distance_nm = 0.0;
  // Procedure heading at the fix in degrees [0, 360), or kNoBearing when unknown.
  // For a SID this is the INBOUND heading (direction the procedure arrives at
  // the fix from the runway side); for a STAR the OUTBOUND heading (direction
  // it leaves the fix toward the runway). Drives the turn-angle penalty at the
  // SID-exit / STAR-entry handoff. For an approach or STAR/SID splice proxy
  // goal this is the heading at F toward/from the off-network gate.
  double bearing = kNoBearing;
  // Approach IAF outbound heading [0, 360), or kNoBearing. Set for approach
  // connections (on-network or proxy); used for route metadata, not the turn penalty.
  double approach_bearing = kNoBearing;
  // Off-network published gate to insert between the proxy fix and the airport
  // in MakeRoute; kNoVertex when the search endpoint is already the gate (or
  // there is no gate to file, e.g. approach DCT-to-IAF).
  int splice_vertex = kNoVertex;
  // Great-circle |F→gate| folded into seed_distance_nm when splice_vertex is
  // set; 0 otherwise. Phase split reports procedure body only
  // (seed − splice_leg) as dep/arr; the DCT splice falls into enroute.
  double splice_leg_nm = 0.0;
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
// independent conditions. When every published gate resolves but is off-network,
// BuildStarSpliceArrival / BuildSidSpliceDeparture expose proxy on-network
// goals and MakeRoute inserts the gate into the filed string (issue #30). An
// unresolvable gate is skipped. Airports with no usable procedures at all use
// BuildDctFallback.
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

  // Arrival connections from approach IAFs when the airport has no STAR.
  // Gate-only (path_term IF). On-network inbound IAFs are ordinary goals;
  // off-network IAFs contribute K nearest inbound proxy goals (seed folds
  // |F→I| into the cost — no CSR mutation). Seed body is IAF→MAPT
  // (transition∥final splice) plus the MAPT→airport stub.
  static std::vector<Connection> BuildApproachArrival(const CifpData& cifp,
                                                      const Coordinate& airport_coord,
                                                      const GraphBuilder& builder,
                                                      const std::string& runway_filter);

  // Arrival connections for published STAR IFs that resolve but are off-network
  // (no inbound edge). Each IF contributes K nearest inbound proxy goals F with
  // seed |F→IF| + STAR body (IF→last fix + stub); splice_vertex = IF. Call when
  // BuildArrival's on-network gates are empty. Unresolvable IFs are skipped.
  // Optional `name_filter` restricts to a named STAR (bare name or NAME.TRANSITION)
  // so --star rebinds splice metadata to the requested procedure only.
  static std::vector<Connection> BuildStarSpliceArrival(const CifpData& cifp,
                                                        const Coordinate& airport_coord,
                                                        const GraphBuilder& builder,
                                                        const std::string& runway_filter,
                                                        const std::string& name_filter = {});

  // Departure connections for published SID exits that resolve but are
  // off-network (no outbound edge). Each exit contributes K nearest outbound
  // proxy goals F with seed SID body (runway→exit) + |exit→F|; splice_vertex =
  // exit. Call when BuildDeparture's on-network gates are empty.
  // Optional `name_filter` restricts to a named SID (same selector shape as --sid).
  static std::vector<Connection> BuildSidSpliceDeparture(const CifpData& cifp,
                                                         const Coordinate& airport_coord,
                                                         const GraphBuilder& builder,
                                                         const std::string& runway_filter,
                                                         const std::string& name_filter = {});

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
