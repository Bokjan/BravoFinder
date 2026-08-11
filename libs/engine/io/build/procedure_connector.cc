// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/build/procedure_connector.h"

#include <algorithm>

#include "core/domain/coordinate.h"
#include "io/build/graph_builder.h"

namespace bf {

namespace {

// Resolve a procedure leg's fix to a graph vertex by its full (ident, region)
// key. Returns kNoVertex when the leg has no resolvable fix. The ident-only fallback
// was removed: a procedure leg carries its region, so resolving by
// (ident, region) is both correct and unambiguous, and silently guessing a
// region would risk wiring a procedure to the wrong fix.
int ResolveFix(const ProcedureLeg& leg, const GraphBuilder& builder) {
  if (leg.fix.IdentView().empty()) {
    return kNoVertex;
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

ProcedureRef MakeApproachRef(const Procedure& p, std::string_view iaf) {
  ProcedureRef ref = MakeRef(p);
  ref.iaf = std::string(iaf);
  return ref;
}

// Merge a (fix_vertex, seed, bearing, ref[, splice]) finding into the connection
// list, keeping the smallest seed distance per fix (and its matching bearing /
// splice metadata) and collecting every procedure ref. When the seed wins, the
// winning ref is moved to procedures.front() so SelectProcedures / metadata pick
// the priced option. A vector is enough: real airports expose at most a few dozen
// distinct handoff fixes per side (p99 ~14, max 35 in cycle 2601), so a linear
// scan beats a hash map and avoids per-airport bucket allocation.
void Accumulate(std::vector<Connection>& by_fix, int fix_vertex, double seed, double bearing,
                const ProcedureRef& ref, double approach_bearing = kNoBearing,
                int splice_vertex = kNoVertex, double splice_leg_nm = 0.0) {
  for (Connection& c : by_fix) {
    if (c.fix_vertex != fix_vertex) {
      continue;
    }
    c.procedures.push_back(ref);
    if (seed < c.seed_distance_nm) {
      c.seed_distance_nm = seed;
      c.bearing = bearing;
      c.approach_bearing = approach_bearing;
      c.splice_vertex = splice_vertex;
      c.splice_leg_nm = splice_leg_nm;
      std::swap(c.procedures.front(), c.procedures.back());
    }
    return;
  }
  Connection c;
  c.fix_vertex = fix_vertex;
  c.seed_distance_nm = seed;
  c.bearing = bearing;
  c.approach_bearing = approach_bearing;
  c.splice_vertex = splice_vertex;
  c.splice_leg_nm = splice_leg_nm;
  c.procedures.push_back(ref);
  by_fix.push_back(std::move(c));
}

// One on-network fix a procedure record passes, with the polyline distance from
// the record's start accumulated up to it and whether it is the record's
// PUBLISHED handoff point (see WalkOnNetworkFixes for what makes a fix a gate).
struct FixHit {
  int vertex;
  double cumulative_nm;
  bool is_gate;
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

// Index of the record's last leg that terminates at a resolvable fix, or
// p.legs.size() when it has none. This is the SID's published exit: the fix the
// procedure leaves the aircraft at, matching the bold transition-end fix on a
// chart. A SID's own IF legs cannot serve here -- an IF marks where a TRANSITION
// begins (the fix a branch forks from), which is the wrong end of the record.
size_t LastFixBearingLeg(const Procedure& p) {
  size_t last = p.legs.size();
  for (size_t i = 0; i < p.legs.size(); ++i) {
    if (p.legs[i].fix_is_definite()) {
      last = i;
    }
  }
  return last;
}

WalkResult WalkOnNetworkFixes(const Procedure& p, const GraphBuilder& builder, WalkDir dir) {
  WalkResult result;
  // The SID's published exit: its last fix-bearing leg (a STAR needs no such
  // index -- it is entered at its IF, identified by path terminator below).
  // Legs other than this one are flown THROUGH, not handed off at.
  const size_t sid_exit_leg = dir == WalkDir::kOutbound ? LastFixBearingLeg(p) : p.legs.size();
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
  for (size_t i = 0; i < p.legs.size(); ++i) {
    const ProcedureLeg& leg = p.legs[i];
    const int v = leg.fix_is_definite() ? ResolveFix(leg, builder) : kNoVertex;
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
      const bool gate =
          dir == WalkDir::kInbound ? leg.path_term == PathTerminator::kIF : i == sid_exit_leg;
      result.hits.push_back(FixHit{v, cumulative, gate});
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

std::vector<Connection> Finalize(std::vector<Connection> by_fix) {
  // Stable ordering by seed distance keeps results deterministic.
  std::sort(by_fix.begin(), by_fix.end(), [](const Connection& a, const Connection& b) {
    if (a.seed_distance_nm != b.seed_distance_nm) {
      return a.seed_distance_nm < b.seed_distance_nm;
    }
    return a.fix_vertex < b.fix_vertex;
  });
  return by_fix;
}

// Collect one side's connections: walk every matching procedure record and
// accumulate its handoff fixes. `gate_only` keeps only the published handoff
// points (the STAR's Initial Fix / the SID's last fix-bearing leg). Callers
// that need the legacy "any on-network fix" model pass false; BuildSide always
// uses gate_only=true (issue #30 removed the airport-level mid-join fallback).
//
// Bearings are computed from the FULL hit list even when only gates survive: a
// gate's procedure heading is set by the fix that precedes/follows it along the
// published track, not by the next gate.
std::vector<Connection> CollectSide(const CifpData& cifp, ProcedureType want, WalkDir dir,
                                    const Coordinate& airport_coord, const GraphBuilder& builder,
                                    const std::string& runway_filter, bool gate_only) {
  const bool departure = dir == WalkDir::kOutbound;
  std::vector<Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != want || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const WalkResult walk = WalkOnNetworkFixes(p, builder, dir);
    if (walk.hits.empty()) {
      continue;
    }
    // The unmeasured runway-to-first-fix (departure) / last-fix-to-runway
    // (arrival) portion, bridged with a straight line.
    const double runway_bridge =
        departure ? (walk.have_first ? airport_coord.DistanceTo(walk.first_coord) : 0.0)
                  : (walk.have_last ? walk.last_coord.DistanceTo(airport_coord) : 0.0);
    std::vector<Coordinate> hit_coords;
    hit_coords.reserve(walk.hits.size());
    for (const FixHit& h : walk.hits) {
      hit_coords.push_back(builder.graph().CoordOf(h.vertex));
    }
    for (size_t j = 0; j < walk.hits.size(); ++j) {
      if (gate_only && !walk.hits[j].is_gate) {
        continue;
      }
      double seed = 0.0;
      double bearing = 0.0;
      if (departure) {
        seed = runway_bridge + walk.hits[j].cumulative_nm;
        // Inbound heading: from the previous resolved fix (the runway side) into
        // the hit. For the first hit that predecessor is the airport, matching
        // the runway-bridge geometry the seed uses.
        const Coordinate& prev = (j == 0) ? airport_coord : hit_coords[j - 1];
        bearing = prev.BearingTo(hit_coords[j]);
      } else {
        seed = (walk.total_nm - walk.hits[j].cumulative_nm) + runway_bridge;
        // Outbound heading: from the hit toward the next resolved fix on the way
        // to the runway. For the last hit that successor is the airport.
        const Coordinate& next = (j + 1 < walk.hits.size()) ? hit_coords[j + 1] : airport_coord;
        bearing = hit_coords[j].BearingTo(next);
      }
      Accumulate(by_fix, walk.hits[j].vertex, seed, bearing, MakeRef(p));
    }
  }
  return by_fix;
}

// Collect one side's published on-network gates only. Airports whose STAR/SID
// gates are all off-network are handled by BuildStarSpliceArrival /
// BuildSidSpliceDeparture (issue #30) rather than the former airport-level
// mid-join fallback (joining at an on-network fix the procedure merely passes).
std::vector<Connection> BuildSide(const CifpData& cifp, ProcedureType want, WalkDir dir,
                                  const Coordinate& airport_coord, const GraphBuilder& builder,
                                  const std::string& runway_filter) {
  return Finalize(CollectSide(cifp, want, dir, airport_coord, builder, runway_filter,
                              /*gate_only=*/true));
}

// Polyline distance along a procedure's definite fixes, optionally starting at
// a given vertex and stopping at (and including) the MAPT leg. No-fix legs
// contribute their published distance_nm when present (same pending rule as
// WalkOnNetworkFixes). Returns false when `start_vertex` is set but never found.
struct ApproachWalk {
  double nm = 0.0;
  Coordinate end_coord{};
  bool have_end = false;
  // First definite fix after the start (for outbound bearing at an IAF).
  Coordinate next_coord{};
  bool have_next = false;
};

bool WalkApproachSegment(const Procedure& p, const GraphBuilder& builder, int start_vertex,
                         bool stop_at_mapt, ApproachWalk& out) {
  double cumulative = 0.0;
  bool measuring = (start_vertex < 0);
  bool have_prev = false;
  Coordinate prev_coord{};
  double pending_nm = 0.0;
  bool found_start = (start_vertex < 0);

  for (const ProcedureLeg& leg : p.legs) {
    const int v = leg.fix_is_definite() ? ResolveFix(leg, builder) : kNoVertex;
    if (v < 0) {
      if (measuring && leg.distance_nm > 0.0) {
        pending_nm += leg.distance_nm;
      }
      continue;
    }
    const Coordinate this_coord = builder.graph().CoordOf(v);
    if (!measuring) {
      if (v == start_vertex) {
        measuring = true;
        found_start = true;
        prev_coord = this_coord;
        have_prev = true;
        pending_nm = 0.0;
        // Starting at this fix: do not count a span yet; look ahead for bearing.
        if (leg.is_mapt && stop_at_mapt) {
          out.nm = 0.0;
          out.end_coord = this_coord;
          out.have_end = true;
          return true;
        }
        continue;
      }
      continue;
    }
    if (have_prev) {
      cumulative += pending_nm > 0.0 ? pending_nm : prev_coord.DistanceTo(this_coord);
      if (!out.have_next) {
        out.next_coord = this_coord;
        out.have_next = true;
      }
    }
    pending_nm = 0.0;
    prev_coord = this_coord;
    have_prev = true;
    out.end_coord = this_coord;
    out.have_end = true;
    out.nm = cumulative;
    if (leg.is_mapt && stop_at_mapt) {
      return found_start;
    }
  }
  return found_start;
}

// Empty-transition approach record with the same name (final + missed segment).
const Procedure* FindApproachFinal(const CifpData& cifp, const std::string& name) {
  for (const Procedure& p : cifp.procedures) {
    if (p.type == ProcedureType::kApproach && p.name == name && p.transition_ident.empty()) {
      return &p;
    }
  }
  return nullptr;
}

// Seed body from an IAF gate through MAPT (splicing transition ∥ final) plus
// the MAPT→airport stub. `gate_proc` is the record that owns the IF; `gate_v`
// is the IAF vertex (on- or off-network). Does not include any |F→I| proxy leg.
double ApproachBodyAndStubNm(const Procedure& gate_proc, int gate_v, const CifpData& cifp,
                             const Coordinate& airport_coord, const GraphBuilder& builder,
                             double& bearing_from_iaf_out) {
  ApproachWalk from_gate;
  // Transition record: IAF → end of transition (usually the final IF).
  // Final record: start → MAPT. When gate_proc itself is the final (empty
  // transition), a single walk IAF→MAPT covers the body.
  const bool gate_is_final = gate_proc.transition_ident.empty();
  if (!WalkApproachSegment(gate_proc, builder, gate_v, /*stop_at_mapt=*/gate_is_final, from_gate)) {
    return airport_coord.DistanceTo(builder.graph().CoordOf(gate_v));  // degenerate
  }

  double body = from_gate.nm;
  Coordinate end = from_gate.have_end ? from_gate.end_coord : builder.graph().CoordOf(gate_v);
  if (from_gate.have_next) {
    bearing_from_iaf_out = builder.graph().CoordOf(gate_v).BearingTo(from_gate.next_coord);
  } else {
    bearing_from_iaf_out = builder.graph().CoordOf(gate_v).BearingTo(airport_coord);
  }

  if (!gate_is_final) {
    const Procedure* final_proc = FindApproachFinal(cifp, gate_proc.name);
    if (final_proc != nullptr) {
      ApproachWalk final_walk;
      // Start from the beginning of the final record (start_vertex < 0) and
      // stop at MAPT. WalkApproachSegment from start measures the final
      // first-definite→MAPT polyline (the first fix contributes 0 nm).
      //
      // Implicit splice assumption: the transition's last definite fix equals
      // the final's first definite fix (typically the final IF). When that
      // holds, body = (IAF→IF) + (IF→MAPT) with no double-count and no gap.
      // Cycle-2601 X-Plane CIFP audit (~65k named approach transitions): the
      // assumption holds for ~95% of records, including every transition at
      // the major hubs sampled (KLAX/KJFK/KDEN/…/KMFR/KTVL). The remaining
      // ~5% are NOT a single "missed gap" pathology — they split into:
      //   (a) PT / hold-at-FAF: transition ends at a fix that appears LATER
      //       in the final (FAF), so the final IF is upstream of the splice;
      //       a naive GC(transition_end → final_IF) would add a reverse leg
      //       and then still walk IF→FAF, over-counting.
      //   (b) True disjoint (arc/VI intercept, etc.): transition end never
      //       appears on the final — here a GC gap would under-count less,
      //       but only this subset benefits.
      // Seed is an approximation used only for goal pricing (not routing
      // feasibility), so behavior stays "add the two walks" and accepts the
      // rare under/over-count rather than a one-sided gap fill that worsens
      // (a). A pattern-aware splice (walk final from the overlapping fix when
      // present; else add GC to the final IF) is the right follow-up if seed
      // bias at no-STAR PT/arc airports becomes user-visible.
      if (WalkApproachSegment(*final_proc, builder, /*start_vertex=*/kNoVertex,
                              /*stop_at_mapt=*/true, final_walk) &&
          final_walk.have_end) {
        body += final_walk.nm;
        end = final_walk.end_coord;
      }
    }
  }

  return body + end.DistanceTo(airport_coord);
}

constexpr int kApproachProxyK = 5;
// Same K as approach proxies: nearest on-network fixes around an off-net
// published STAR IF / SID exit (issue #30 DCT-splice).
constexpr int kSpliceProxyK = 5;

// STAR body from a published IF through the record's last definite fix, plus
// the last-fix→airport stub. No MAPT cutoff — STARs are not approaches.
double StarBodyAndStubNm(const Procedure& p, int gate_v, const Coordinate& airport_coord,
                         const GraphBuilder& builder) {
  ApproachWalk from_gate;
  if (!WalkApproachSegment(p, builder, gate_v, /*stop_at_mapt=*/false, from_gate)) {
    return airport_coord.DistanceTo(builder.graph().CoordOf(gate_v));
  }
  const Coordinate end = from_gate.have_end ? from_gate.end_coord : builder.graph().CoordOf(gate_v);
  return from_gate.nm + end.DistanceTo(airport_coord);
}

// SID body from the airport to the published exit: runway→first-fix bridge plus
// the first→exit polyline (exit is the last definite fix).
double SidBodyToExitNm(const Procedure& p, int exit_v, const Coordinate& airport_coord,
                       const GraphBuilder& builder) {
  ApproachWalk body;
  if (!WalkApproachSegment(p, builder, /*start_vertex=*/kNoVertex, /*stop_at_mapt=*/false, body) ||
      !body.have_end) {
    return airport_coord.DistanceTo(builder.graph().CoordOf(exit_v));
  }
  // Recover the first definite fix for the runway bridge (WalkApproachSegment
  // from the start measures first→last with the first contributing 0 nm).
  Coordinate first = builder.graph().CoordOf(exit_v);
  for (const ProcedureLeg& leg : p.legs) {
    if (!leg.fix_is_definite()) {
      continue;
    }
    const int v = ResolveFix(leg, builder);
    if (v >= 0) {
      first = builder.graph().CoordOf(v);
      break;
    }
  }
  return airport_coord.DistanceTo(first) + body.nm;
}

// Off-network published STAR IFs → proxy goals (splice_vertex = IF).
std::vector<Connection> CollectStarSpliceArrivals(const CifpData& cifp,
                                                  const Coordinate& airport_coord,
                                                  const GraphBuilder& builder,
                                                  const std::string& runway_filter) {
  std::vector<Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kStar || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    for (const ProcedureLeg& leg : p.legs) {
      if (leg.path_term != PathTerminator::kIF || !leg.fix_is_definite()) {
        continue;
      }
      const int gate = ResolveFix(leg, builder);
      if (gate < 0) {
        continue;  // no vertex — cannot proxy (same as approach)
      }
      if (builder.HasInbound(gate)) {
        continue;  // on-network gate — BuildArrival owns it
      }
      const double body_stub = StarBodyAndStubNm(p, gate, airport_coord, builder);
      const ProcedureRef pref = MakeRef(p);
      const Coordinate gate_coord = builder.graph().CoordOf(gate);
      for (int f : builder.NearestOnNetwork(gate_coord, kSpliceProxyK, /*inbound=*/true)) {
        if (f == gate) {
          continue;
        }
        const Coordinate f_coord = builder.graph().CoordOf(f);
        const double splice_leg = f_coord.DistanceTo(gate_coord);
        const double seed = splice_leg + body_stub;
        const double bearing = f_coord.BearingTo(gate_coord);
        Accumulate(by_fix, f, seed, bearing, pref, kNoBearing, gate, splice_leg);
      }
    }
  }
  return by_fix;
}

// Off-network published SID exits → proxy goals (splice_vertex = exit).
std::vector<Connection> CollectSidSpliceDepartures(const CifpData& cifp,
                                                   const Coordinate& airport_coord,
                                                   const GraphBuilder& builder,
                                                   const std::string& runway_filter) {
  std::vector<Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kSid || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const size_t exit_leg = LastFixBearingLeg(p);
    if (exit_leg >= p.legs.size()) {
      continue;  // radar-vector / no fix-bearing exit — no splice candidate
    }
    const int exit_v = ResolveFix(p.legs[exit_leg], builder);
    if (exit_v < 0) {
      continue;
    }
    if (builder.HasOutbound(exit_v)) {
      continue;  // on-network gate — BuildDeparture owns it
    }
    const double body = SidBodyToExitNm(p, exit_v, airport_coord, builder);
    const ProcedureRef pref = MakeRef(p);
    const Coordinate exit_coord = builder.graph().CoordOf(exit_v);
    for (int f : builder.NearestOnNetwork(exit_coord, kSpliceProxyK, /*inbound=*/false)) {
      if (f == exit_v) {
        continue;
      }
      const Coordinate f_coord = builder.graph().CoordOf(f);
      const double splice_leg = exit_coord.DistanceTo(f_coord);
      const double seed = body + splice_leg;
      // Inbound heading at the proxy: arriving from the exit along the DCT splice.
      const double bearing = exit_coord.BearingTo(f_coord);
      Accumulate(by_fix, f, seed, bearing, pref, kNoBearing, exit_v, splice_leg);
    }
  }
  return by_fix;
}

// Approach IAF connections for airports with no STAR: gate-only (path_term IF).
// On-network inbound IAFs become search goals directly; off-network IAFs that
// still resolve to a vertex are represented by K nearest inbound on-network
// proxies F with seed |F→I| + (I→MAPT…) + stub (no graph mutation).
std::vector<Connection> CollectApproachArrivals(const CifpData& cifp,
                                                const Coordinate& airport_coord,
                                                const GraphBuilder& builder,
                                                const std::string& runway_filter) {
  std::vector<Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kApproach || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    for (const ProcedureLeg& leg : p.legs) {
      if (leg.path_term != PathTerminator::kIF || !leg.fix_is_definite()) {
        continue;
      }
      const int iaf = ResolveFix(leg, builder);
      if (iaf < 0) {
        continue;  // no vertex — cannot price or proxy
      }
      double bearing_from_iaf = kNoBearing;
      const double body_stub =
          ApproachBodyAndStubNm(p, iaf, cifp, airport_coord, builder, bearing_from_iaf);
      const std::string iaf_ident(leg.fix.IdentView());
      const ProcedureRef pref = MakeApproachRef(p, iaf_ident);
      if (builder.HasInbound(iaf)) {
        Accumulate(by_fix, iaf, body_stub, bearing_from_iaf, pref, bearing_from_iaf);
        continue;
      }
      // Proxy goals: nearest on-network inbound fixes around the off-net IAF.
      const Coordinate iaf_coord = builder.graph().CoordOf(iaf);
      for (int f : builder.NearestOnNetwork(iaf_coord, kApproachProxyK, /*inbound=*/true)) {
        if (f == iaf) {
          continue;
        }
        const Coordinate f_coord = builder.graph().CoordOf(f);
        const double seed = f_coord.DistanceTo(iaf_coord) + body_stub;
        // Turn at F is priced as leaving F toward the IAF (the virtual first leg).
        const double bearing = f_coord.BearingTo(iaf_coord);
        Accumulate(by_fix, f, seed, bearing, pref, bearing_from_iaf);
      }
    }
  }
  return by_fix;
}

}  // namespace

std::vector<Connection> ProcedureConnector::BuildDeparture(const CifpData& cifp,
                                                           const Coordinate& airport_coord,
                                                           const GraphBuilder& builder,
                                                           const std::string& runway_filter) {
  // The seed is the estimated distance flown from the runway to the exit fix:
  // the straight line from the airport to the record's first resolved fix (the
  // unmeasured runway-to-first-fix portion) plus the record's own polyline up to
  // the exit. The polyline follows the published track, so an exit reached only
  // after a long detour gets a larger (more honest) seed than its straight-line
  // distance would suggest.
  return BuildSide(cifp, ProcedureType::kSid, WalkDir::kOutbound, airport_coord, builder,
                   runway_filter);
}

std::vector<Connection> ProcedureConnector::BuildArrival(const CifpData& cifp,
                                                         const Coordinate& airport_coord,
                                                         const GraphBuilder& builder,
                                                         const std::string& runway_filter) {
  // The seed is the estimated distance from the entry fix to the runway: the
  // record's polyline from the entry to the last resolved fix, plus the straight
  // line from there to the airport (the unmeasured last-fix-to-runway portion).
  // Following the published track makes a far entry fix (e.g. KLAX BASET5 via
  // PGS, ~260 NM out) cost more than a nearer entry on another STAR, so the
  // search prefers the latter.
  return BuildSide(cifp, ProcedureType::kStar, WalkDir::kInbound, airport_coord, builder,
                   runway_filter);
}

std::vector<Connection> ProcedureConnector::BuildApproachArrival(const CifpData& cifp,
                                                                 const Coordinate& airport_coord,
                                                                 const GraphBuilder& builder,
                                                                 const std::string& runway_filter) {
  // No STAR: connect via approach IAFs (IF legs). On-network inbound IAFs are
  // ordinary goals; off-network IAFs contribute proxy goals at nearby inbound
  // fixes (seed folds |F→I| into the cost — no virtual edges on the CSR).
  //
  // TODO(perf): Recomputed on every FindRoutes for airports with an empty STAR
  // set (KTVL/KMFR/04TT-class). Cost is in-memory only (walk approach legs +
  // NearestOnNetwork via DegreeGrid, sub-ms to a few ms) — not a measured
  // hotspot today, and airports that publish STARs never enter this path.
  // If profiling later shows no-STAR airports on the HTTP/MCP hot path, cache
  // by airport (optionally + runway_filter) with the same double-checked
  // locking + unique_ptr pattern as procedure_cache_, keeping FindRoutes const
  // and concurrently safe; any such cache must pass the tsan preset.
  std::vector<Connection> by_fix =
      CollectApproachArrivals(cifp, airport_coord, builder, runway_filter);
  return Finalize(std::move(by_fix));
}

std::vector<Connection> ProcedureConnector::BuildStarSpliceArrival(
    const CifpData& cifp, const Coordinate& airport_coord, const GraphBuilder& builder,
    const std::string& runway_filter) {
  std::vector<Connection> by_fix =
      CollectStarSpliceArrivals(cifp, airport_coord, builder, runway_filter);
  return Finalize(std::move(by_fix));
}

std::vector<Connection> ProcedureConnector::BuildSidSpliceDeparture(
    const CifpData& cifp, const Coordinate& airport_coord, const GraphBuilder& builder,
    const std::string& runway_filter) {
  std::vector<Connection> by_fix =
      CollectSidSpliceDepartures(cifp, airport_coord, builder, runway_filter);
  return Finalize(std::move(by_fix));
}

std::vector<Connection> ProcedureConnector::BuildDctFallback(const Coordinate& airport_coord,
                                                             const GraphBuilder& builder, int count,
                                                             bool arrival) {
  // No gate restriction applies here: the gate rule exists so an enroute airway
  // cannot bypass a published procedure body by joining it near the threshold,
  // and a DCT connection has no procedure body to bypass. It is also mutually
  // exclusive with procedure connections (see nav_database_routing.cc) -- this
  // runs only when the airport exposed none -- so a near-field DCT fix is a
  // legitimate direct join by great-circle distance.
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
