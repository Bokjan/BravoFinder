// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/procedure_connector.h"

#include <algorithm>
#include <unordered_map>

#include "core/domain/coordinate.h"
#include "io/graph_builder.h"

namespace bf {

namespace {

// Resolve a procedure leg's fix to a graph vertex by its full (ident, region)
// key. Returns -1 when the leg has no resolvable fix. The ident-only fallback
// was removed: a procedure leg carries its region, so resolving by
// (ident, region) is both correct and unambiguous, and silently guessing a
// region would risk wiring a procedure to the wrong fix.
int ResolveFix(const ProcedureLeg& leg, const GraphBuilder& builder) {
  if (leg.fix.IdentView().empty()) {
    return -1;
  }
  return builder.VertexByIdent(leg.fix);
}

// Whether this procedure record should be considered, honoring an optional
// runway filter. An empty filter accepts all; otherwise the procedure must be
// for that runway (or be runway-independent, i.e. carry no runway).
bool RunwayMatches(const Procedure& p, const std::string& runway_filter) {
  if (runway_filter.empty()) {
    return true;
  }
  return p.runway.empty() || p.runway == runway_filter;
}

// Build a ProcedureRef describing one procedure record.
ProcedureRef MakeRef(const Procedure& p) {
  ProcedureRef ref;
  ref.type = p.type;
  ref.name = p.name;
  ref.transition = p.transition_ident;
  ref.runway = p.runway;
  return ref;
}

// Merge a (fix_vertex, seed, bearing, ref) finding into the connection map,
// keeping the smallest seed distance per fix (and its matching bearing) and
// collecting every procedure ref.
void Accumulate(std::unordered_map<int, Connection>& by_fix, int fix_vertex, double seed,
                double bearing, const ProcedureRef& ref) {
  auto it = by_fix.find(fix_vertex);
  if (it == by_fix.end()) {
    Connection c;
    c.fix_vertex = fix_vertex;
    c.seed_distance_nm = seed;
    c.bearing = bearing;
    c.procedures.push_back(ref);
    by_fix.emplace(fix_vertex, std::move(c));
    return;
  }
  it->second.procedures.push_back(ref);
  if (seed < it->second.seed_distance_nm) {
    it->second.seed_distance_nm = seed;
    it->second.bearing = bearing;
  }
}

// One on-network fix a procedure record passes, with the polyline distance from
// the record's start accumulated up to it.
struct FixHit {
  int vertex;
  double cumulative_nm;
};

// The result of walking one procedure record's legs: every on-network fix it
// reaches, the total polyline length of the record, and the first/last resolved
// fix coordinates (used to bridge the record's endpoints to the airport with a
// straight line, since the runway-to-first-fix and last-fix-to-runway portions
// are not measured here).
struct WalkResult {
  std::vector<FixHit> hits;
  double total_nm = 0.0;
  Coordinate first_coord{};
  Coordinate last_coord{};
  bool have_first = false;
  bool have_last = false;
};

// Which airway-edge direction makes a fix a usable procedure connection point.
// A SID hands the aircraft to a fix it then departs along an airway (needs an
// outbound edge); a STAR picks the aircraft up at a fix reached along an airway
// (needs an inbound edge). A forward-only airway that dead-ends at a STAR entry
// gate leaves that fix inbound-only, so the two sides must not share one test.
enum class WalkDir { kOutbound, kInbound };

WalkResult WalkOnNetworkFixes(const Procedure& p, const GraphBuilder& builder, WalkDir dir) {
  WalkResult result;
  // `cumulative` measures the polyline from the FIRST resolved fix to the
  // current one. The runway-to-first-fix portion is covered separately by the
  // runway bridge (a straight line to first_coord), so the legs closing that
  // span are not added here -- counting them would double-count it.
  double cumulative = 0.0;
  bool have_prev = false;
  Coordinate prev_coord{};
  // Distance accrued by no-fix legs (heading/altitude/arc, no resolvable end
  // fix) since the last resolved fix. It stands in for the along-track length of
  // the gap the next resolved fix closes, so the gap is counted once -- via this
  // accrued distance -- rather than twice (this AND the great-circle to the next
  // fix). Legs before the first fix accrue here too but are spent nowhere: the
  // first fix resets pending without spending it (the bridge already covers that
  // runway-to-first-fix span).
  double pending_nm = 0.0;
  for (const ProcedureLeg& leg : p.legs) {
    const int v = leg.fix_is_definite() ? ResolveFix(leg, builder) : -1;
    if (v < 0) {
      if (leg.distance_nm > 0.0) {
        pending_nm += leg.distance_nm;
      }
      continue;
    }
    const Coordinate this_coord = builder.graph().CoordOf(v);
    if (have_prev) {
      // Count the prev->this span exactly once: the accrued no-fix distance if
      // any legs crossed it, else the direct great-circle. Adding both would
      // double-count the same geographic span.
      cumulative += pending_nm > 0.0 ? pending_nm : prev_coord.DistanceTo(this_coord);
    }
    pending_nm = 0.0;
    const bool usable = dir == WalkDir::kInbound ? builder.HasInbound(v) : builder.HasOutbound(v);
    if (usable) {
      result.hits.push_back(FixHit{v, cumulative});
    }
    if (!result.have_first) {
      result.first_coord = this_coord;
      result.have_first = true;
    }
    result.last_coord = this_coord;
    result.have_last = true;
    prev_coord = this_coord;
    have_prev = true;
  }
  result.total_nm = cumulative;
  return result;
}

std::vector<Connection> Finalize(std::unordered_map<int, Connection>& by_fix) {
  std::vector<Connection> out;
  out.reserve(by_fix.size());
  for (auto& [vertex, conn] : by_fix) {
    out.push_back(std::move(conn));
  }
  // Stable ordering by seed distance keeps results deterministic.
  std::sort(out.begin(), out.end(), [](const Connection& a, const Connection& b) {
    if (a.seed_distance_nm != b.seed_distance_nm) {
      return a.seed_distance_nm < b.seed_distance_nm;
    }
    return a.fix_vertex < b.fix_vertex;
  });
  return out;
}

// Gap-gated doorstep filter, shared by both procedure sides.
//
// A seed is the estimated procedure distance flown between the runway and the
// connection fix (see Build{Departure,Arrival}). A fix seeded at ~0 NM sits on
// the runway threshold itself: seeding the search there lets the enroute network
// fly straight to the airport door and reduces the SID/STAR to a zero-length
// stub (the KJFK->YSSY "...B450 TESAT STAR YSSY", arr 0.3 NM case). Such a fix
// must not be a connection point.
//
// But a small seed alone does not prove degeneracy: some airports legitimately
// have their only close entry at 2-5 NM (a genuine short final), and dropping it
// would force a 60-150 NM detour. The distinguishing signal is the SECOND entry:
// a degenerate "airway reached the doorstep" airport still exposes the real
// procedure body at a moderate distance, whereas a genuine short-final airport
// has one close entry then a large jump. So we drop the doorstep entries only
// when a fallback entry exists in [kNearSeedNm, kFallbackSeedNm] -- a normal
// procedure-body length. This never empties the set: the surviving fallback is
// what licenses the drop.
//
// Thresholds are empirical (the seed distribution is continuous, with no natural
// gap; see .notes/research/2026-07-29_arrival_star_connection.md §8):
//   kNearSeedNm     1.0  -- below this a fix is effectively on the threshold;
//                           the degenerate cluster lives here.
//   kFallbackSeedNm 20.0 -- an upper bound on a normal SID/STAR body; a fallback
//                           within it confirms a real procedure was bypassed.
//                           Chosen over 15 to also catch busy airports whose real
//                           body sits at 15-20 NM (e.g. RPLL, LIRQ, KFLL, LLBG),
//                           with zero false positives observed across cycle 2601.
constexpr double kNearSeedNm = 1.0;
constexpr double kFallbackSeedNm = 20.0;

void DropDoorstepConnections(std::unordered_map<int, Connection>& by_fix) {
  bool has_doorstep = false;
  bool has_fallback = false;
  for (const auto& [vertex, conn] : by_fix) {
    if (conn.seed_distance_nm < kNearSeedNm) {
      has_doorstep = true;
    } else if (conn.seed_distance_nm <= kFallbackSeedNm) {
      has_fallback = true;
    }
  }
  if (!has_doorstep || !has_fallback) {
    return;
  }
  for (auto it = by_fix.begin(); it != by_fix.end();) {
    if (it->second.seed_distance_nm < kNearSeedNm) {
      it = by_fix.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace

std::vector<Connection> ProcedureConnector::BuildDeparture(const CifpData& cifp,
                                                           const Coordinate& airport_coord,
                                                           const GraphBuilder& builder,
                                                           const std::string& runway_filter) {
  // A SID can hand the aircraft to the network at ANY on-network fix it passes,
  // not just its last one: filing "join the airway at <fix>" is routine. Every
  // such fix is exposed as a candidate connection and the multi-source search
  // picks whichever minimizes seed + enroute cost.
  //
  // The seed is the estimated distance flown from the runway to that fix: the
  // straight line from the airport to the record's first resolved fix (the
  // unmeasured runway-to-first-fix portion) plus the record's own polyline up to
  // the candidate. The polyline follows the published track, so a fix reached
  // only after a long detour gets a larger (more honest) seed than its straight
  // -line distance would suggest, steering the search toward closer fixes.
  std::unordered_map<int, Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kSid || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const WalkResult walk = WalkOnNetworkFixes(p, builder, WalkDir::kOutbound);
    if (walk.hits.empty()) {
      continue;
    }
    const double runway_bridge = walk.have_first ? airport_coord.DistanceTo(walk.first_coord) : 0.0;
    // Inbound heading at each hit fix: the bearing from the previous resolved
    // fix (the runway side) into the hit. For the first hit that predecessor is
    // the airport, matching the runway-bridge geometry the seed uses.
    std::vector<Coordinate> hit_coords;
    hit_coords.reserve(walk.hits.size());
    for (const FixHit& h : walk.hits) {
      hit_coords.push_back(builder.graph().CoordOf(h.vertex));
    }
    for (size_t j = 0; j < walk.hits.size(); ++j) {
      const double seed = runway_bridge + walk.hits[j].cumulative_nm;
      const Coordinate& prev = (j == 0) ? airport_coord : hit_coords[j - 1];
      const double bearing = prev.BearingTo(hit_coords[j]);
      Accumulate(by_fix, walk.hits[j].vertex, seed, bearing, MakeRef(p));
    }
  }
  DropDoorstepConnections(by_fix);
  return Finalize(by_fix);
}

std::vector<Connection> ProcedureConnector::BuildArrival(const CifpData& cifp,
                                                         const Coordinate& airport_coord,
                                                         const GraphBuilder& builder,
                                                         const std::string& runway_filter) {
  // A STAR can pick the aircraft up at ANY on-network fix it passes, not just
  // its first one. Every such fix is exposed as a candidate; the multi-source
  // search picks the cheapest entry. The seed is the estimated distance from
  // that fix to the runway: the record's polyline from the candidate to the last
  // resolved fix, plus the straight line from there to the airport (the
  // unmeasured last-fix-to-runway portion). Following the published track makes
  // a far entry fix (e.g. KLAX BASET5 via PGS, ~260 NM out) cost more than a
  // nearer one on the same STAR (e.g. CIVET), so the search prefers the latter.
  std::unordered_map<int, Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kStar || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const WalkResult walk = WalkOnNetworkFixes(p, builder, WalkDir::kInbound);
    if (walk.hits.empty()) {
      continue;
    }
    const double runway_bridge = walk.have_last ? walk.last_coord.DistanceTo(airport_coord) : 0.0;
    // Outbound heading at each hit fix: the bearing from the hit toward the next
    // resolved fix on the way to the runway. For the last hit that successor is
    // the airport, matching the runway-bridge geometry the seed uses.
    std::vector<Coordinate> hit_coords;
    hit_coords.reserve(walk.hits.size());
    for (const FixHit& h : walk.hits) {
      hit_coords.push_back(builder.graph().CoordOf(h.vertex));
    }
    for (size_t j = 0; j < walk.hits.size(); ++j) {
      const double seed = (walk.total_nm - walk.hits[j].cumulative_nm) + runway_bridge;
      const Coordinate& next = (j + 1 < walk.hits.size()) ? hit_coords[j + 1] : airport_coord;
      const double bearing = hit_coords[j].BearingTo(next);
      Accumulate(by_fix, walk.hits[j].vertex, seed, bearing, MakeRef(p));
    }
  }
  DropDoorstepConnections(by_fix);
  return Finalize(by_fix);
}

std::vector<Connection> ProcedureConnector::BuildDctFallback(const Coordinate& airport_coord,
                                                             const GraphBuilder& builder, int count,
                                                             bool arrival) {
  // Intentionally NOT passed through DropDoorstepConnections: the doorstep
  // degeneracy is procedure-specific (an airway reaching the threshold bypasses
  // the STAR/SID body). A DCT fallback fix near the field is a legitimate direct
  // join by great-circle distance with no procedure body to bypass, so a near
  // seed there is correct, not degenerate. It is also mutually exclusive with
  // procedure connections (see nav_database_routing.cc): when procedures exist
  // the filtered set is never emptied, so DCT never re-introduces a doorstep.
  std::vector<Connection> out;
  for (int v : builder.NearestOnNetwork(airport_coord, count, /*inbound=*/arrival)) {
    Connection c;
    c.fix_vertex = v;
    const Coordinate fix_coord = builder.graph().CoordOf(v);
    c.seed_distance_nm = airport_coord.DistanceTo(fix_coord);
    // DCT has no procedure body, so the heading at the fix is the straight-line
    // bearing to/from the airport: inbound for a departure (airport->fix), the
    // direction the aircraft arrives at the fix; outbound for an arrival
    // (fix->airport), the direction it leaves toward the field.
    c.bearing = arrival ? fix_coord.BearingTo(airport_coord) : airport_coord.BearingTo(fix_coord);
    out.push_back(std::move(c));  // no ProcedureRef: this is a DCT connection
  }
  return out;
}

std::vector<SeededEndpoint> ProcedureConnector::ToEndpoints(
    const std::vector<Connection>& connections) {
  std::vector<SeededEndpoint> endpoints;
  endpoints.reserve(connections.size());
  for (const Connection& c : connections) {
    endpoints.push_back(SeededEndpoint{c.fix_vertex, c.seed_distance_nm, c.bearing});
  }
  return endpoints;
}

}  // namespace bf
