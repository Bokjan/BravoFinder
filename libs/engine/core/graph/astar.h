// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "core/constraints/constraint.h"
#include "core/graph/nav_graph.h"
#include "core/routing/route_request.h"

namespace bf {

// The outcome of an A* search.
struct ShortestPath {
  std::vector<int> vertices;  // start..goal inclusive; empty if unreachable
  double distance_nm = 0.0;   // total geographic length (excludes soft penalties)
  double cost = 0.0;          // effective cost = distance + soft penalties
  bool found = false;
};

// Pack a directed edge (from, to) into one 64-bit key. Vertex ids are small
// non-negative ints, so a flat integer key lets a banned-edge set be a sorted
// vector with binary_search -- no std::pair ordering, no hashing, no
// std::function type erasure on the A* hot loop.
inline int64_t EdgeKey(int from, int to) {
  return (static_cast<int64_t>(from) << 32) | static_cast<uint32_t>(to);
}

// An entry in the A* open set, ordered by f = g + h (ascending). Stored in the
// workspace's reusable heap vector so Yen's many spur searches share one
// allocation (cleared per search, capacity retained) instead of each creating
// and growing a fresh priority_queue.
struct QueueNode {
  double f = 0.0;
  int vertex = -1;
  bool operator>(const QueueNode& other) const { return f > other.f; }
};

// A node block filter for the A* hot loop. Combines an optional airport-range
// block (the common "no transit through airports" rule -- airports occupy the
// contiguous tail [airport_first, airport_last) of the vertex range, so this is
// a two-compare range check) with an optional sorted banned-vertex set (Yen's
// per-spur root-node ban). An empty NodeFilter blocks nothing. Blocks() is
// straight-line code (range check + binary_search), inlinable with no
// type-erased call -- this replaces a std::function<bool(int)> that profiling
// showed at ~10% of the multi-source search.
struct NodeFilter {
  int airport_first = -1;                    // first airport vertex (inclusive), or -1 if none
  int airport_last = -1;                     // one-past-last airport vertex (exclusive)
  const std::vector<int>* banned = nullptr;  // sorted ascending, or nullptr

  bool Blocks(int v) const {
    if (airport_first >= 0 && v >= airport_first && v < airport_last) {
      return true;
    }
    return banned != nullptr && std::binary_search(banned->begin(), banned->end(), v);
  }
};

// An edge block filter, the directed-edge counterpart to NodeFilter. Holds an
// optional sorted set of banned EdgeKey values (Yen's per-spur ban). An empty
// EdgeFilter blocks nothing. Replaces a std::function<bool(int,int)>.
struct EdgeFilter {
  const std::vector<int64_t>* banned = nullptr;  // sorted ascending, or nullptr

  bool Blocks(int from, int to) const {
    return banned != nullptr &&
           std::binary_search(banned->begin(), banned->end(), EdgeKey(from, to));
  }
};

// The absolute turn angle between two headings in degrees, normalized to
// [0, 180]: a 350->10 turn is 20, not 340. Used by the turn-angle constraint to
// compare the inbound heading (how the path arrived at a vertex) with the
// outbound heading (the edge leaving it).
inline double TurnAngleDeg(double inbound_deg, double outbound_deg) {
  double delta = std::abs(inbound_deg - outbound_deg);
  if (delta > kDegreesHalfCircle) {
    delta = kDegreesFullCircle - delta;
  }
  return delta;
}

// A soft, path-dependent penalty for sharp turns at path vertices, to suppress
// the near-180-degree "reversals" that arise at SID/STAR handoff fixes when the
// seed-cost-minimizing connection points the wrong way (e.g. a SID exiting at a
// fix with a 150-plus-degree turn onto the first airway). Unlike the per-edge
// Constraint penalties, this is path-dependent -- the cost of leaving v depends
// on how the path entered v -- so it is applied inside the A* relaxation loop
// (and Yen's path re-costing), not via the Constraint/SelectEdge machinery.
//
// This is the "greedy, single-state" form: the inbound heading is read from the
// best-path predecessor (SearchWorkspace::Inbound) rather than expanding the A*
// state to (vertex, heading-bin). It gives up strict global optimality (a
// slightly longer but smoother path to v may have been discarded) in exchange
// for zero state-space growth and ~one atan2 per relaxation. The heuristic
// depends only on the vertex and the penalty is non-negative, so it stays
// admissible/consistent and the closed set remains valid at the greedy-approx
// level. Disabled (the default) adds no trig to the hot path.
//
// Shape (piecewise): zero up to kThresholdDeg; linear to kPenaltyAtKneeNm at
// kKneeDeg; quadratic to kMaxPenaltyNm at 180. The quadratic upper half steepens
// the cost of severe near-reversals (>90) more aggressively than moderate
// course changes, steering the optimizer away from the OC-class reversals while
// barely perturbing normal enroute routing (85% of mid-route turns are <15).
struct TurnPenalty {
  // Calibration constants. Chosen from the turn-angle distribution so the penalty
  // can compete with seed/edge costs (tens to ~200 NM): a ~90 turn maps to ~20 NM,
  // the 157-degree case to ~120 NM, a full 180 to 200 NM. Tunable: revisit after a
  // batch before/after comparison and adjust here only.
  // kMaxPenaltyNm was raised 95 -> 200 after the batch: near-180 reversals are
  // already eliminated at 95, but 200 also suppresses most >135/>150 turns.
  // It was re-measured at cycle-2601 scale after the gate-restricted connection
  // model landed -- 200 was partly standing in for that model, so 95 was expected
  // to suffice afterwards. It does not: at 200 every handoff class still has a
  // shorter tail than at 95 (procedure-connected STAR entries >150: 4.96% vs
  // 5.67%; DCT-fallback STAR entries >150: 0.24% vs 1.02%; SID exits >150: 0.00%
  // vs 0.81%), for +1.4 NM median total distance and no change in route success
  // rate. The reason is that most residual sharp handoffs are DCT-fallback
  // airports with no procedure at all, which the connection model does not touch;
  // 200 remains the calibrated value on its own merits, not as a stand-in.
  static constexpr double kThresholdDeg = 45.0;  // zero below: mid-route turns are nearly all <15
  static constexpr double kKneeDeg = 90.0;       // linear [45,90], quadratic [90,180] knee
  static constexpr double kPenaltyAtKneeNm = 20.0;  // penalty at 90 deg
  static constexpr double kMaxPenaltyNm = 200.0;    // penalty at 180 deg

  bool enabled = false;

  // Penalty in NM for a turn of `angle_deg` (expected in [0, 180]).
  double operator()(double angle_deg) const noexcept {
    if (!enabled || angle_deg <= kThresholdDeg) {
      return 0.0;
    }
    if (angle_deg <= kKneeDeg) {
      // Linear ramp 0 -> kPenaltyAtKneeNm over [kThresholdDeg, kKneeDeg].
      return kPenaltyAtKneeNm * (angle_deg - kThresholdDeg) / (kKneeDeg - kThresholdDeg);
    }
    if (angle_deg >= kDegreesHalfCircle) {
      return kMaxPenaltyNm;
    }
    // Quadratic ramp kPenaltyAtKneeNm -> kMaxPenaltyNm over [kKneeDeg, 180].
    const double t = (angle_deg - kKneeDeg) / (kDegreesHalfCircle - kKneeDeg);
    return kPenaltyAtKneeNm + (kMaxPenaltyNm - kPenaltyAtKneeNm) * t * t;
  }
};

// Optional inputs that shape a search: routing constraints and the bans Yen's
// algorithm uses to carve out alternative paths. All fields are optional; an
// empty SearchOptions reproduces a plain shortest-path search.
struct SearchOptions {
  // Constraints applied to every edge (hard filter + soft penalty). The request
  // they are evaluated against must be supplied when constraints are present.
  std::vector<const Constraint*> constraints;
  const RouteRequest* request = nullptr;

  // Path-dependent soft penalty for sharp turns at vertices.
  // Disabled by default; NavDatabase::FindRoutes enables it. See TurnPenalty.
  TurnPenalty turn_penalty;

  // Yen support: vertices and directed edges that must not be used. These are
  // concrete filters (NodeFilter / EdgeFilter), not std::function, so the hot
  // loop's per-neighbor block check is an inlined range check + binary_search
  // rather than a type-erased call.
  NodeFilter node_filter;
  EdgeFilter edge_filter;
};

// Reusable per-vertex A* scratch, defined below; forward-declared here so the
// reused FindShortestPathMulti overload can take it by reference.
class SearchWorkspace;

// Among the parallel edges from `from` to `to`, return the one the search would
// have traversed: the cheapest ALLOWED edge by effective cost (distance_nm + soft
// penalties), evaluating `options.constraints` exactly as A* relaxation and Yen's
// path re-costing do. Returns nullptr if there is no such edge (none exists, or
// every parallel edge is blocked). Because every constraint is a deterministic
// function of the edge, this reproduces the search's choice, so a route's leg
// labels (airway, distance) match the path the search actually cost -- not the
// merely shortest-by-distance parallel edge. When `out_cost` is non-null it
// receives the selected edge's effective cost (distance_nm + penalties), so a
// caller re-costing a path need not re-evaluate the winner's constraints.
const GraphEdge* SelectEdge(const NavGraph& graph, int from, int to, const SearchOptions& options,
                            double* out_cost = nullptr);

// Find the shortest path from `start` to `goal`. A thin wrapper over the
// multi-source search (a single zero-seed source/goal), so the single- and
// multi-source paths share one A* loop and one heuristic (the chord form) and
// cannot drift apart; unit tests of this entry point exercise the production
// search loop. Soft penalties only add cost, so the heuristic remains a lower
// bound and the result is optimal under the effective (penalized) cost. Returns
// found=false when no path exists.
ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal,
                              const SearchOptions& options);

// Convenience overload: unconstrained shortest path.
ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal);

// A graph vertex pre-loaded with a starting/ending cost, used to attach
// procedures to the enroute network. A SID becomes a source: its connection fix
// with the (estimated) distance flown from the runway to that fix. A STAR
// becomes a goal: its entry fix with the distance flown from there to the
// runway. The connection fixes are ordinary graph vertices, so procedures need
// no graph mutation.
struct SeededEndpoint {
  int vertex = -1;
  double cost = 0.0;  // SID distance (source) or STAR distance (goal), in NM
  // Heading at the connection fix for the turn-angle constraint, in degrees
  // [0, 360), or -1 when unknown (no procedure / a forced via-point). For a
  // source (SID) this is the INBOUND heading -- the direction the procedure
  // arrives at the fix from the runway side. For a goal (STAR) it is the
  // OUTBOUND heading -- the direction the procedure leaves the fix toward the
  // runway. Symmetric: both are "the procedure leg heading at the fix".
  double bearing = -1.0;
};

// Reusable per-vertex scratch for A*. Yen runs the search hundreds of times over
// the same graph (one per spur), and each run only ever touches a tiny fraction
// of the ~10^5 vertices; allocating and O(V)-initializing fresh g/geo/prev/closed
// arrays every time dominated the search cost. Instead the caller keeps one
// workspace and hands it to each search: the arrays are allocated once, and a
// per-search generation stamp makes clearing O(1). A slot whose stamp is not the
// current generation reads as its initial value (g = +inf, prev = -1, not
// closed), so bumping `generation` logically resets everything without touching
// memory. Owned by a single search or a single Yen invocation on the stack -- no
// static/thread_local state -- so distinct concurrent queries never share one,
// keeping the read-only concurrency contract intact.
class SearchWorkspace {
 public:
  // Prepare for a graph of `n` vertices, allocating on first use and growing if a
  // larger graph is seen. Does not touch the value arrays -- the stamp handles
  // logical clearing.
  void Reset(int n);

  // Begin a fresh search: every slot now reads as its initial value. O(1).
  void NextGeneration() { ++generation_; }

  // The A* open set's backing store, reused across spur searches: cleared per
  // search (capacity retained) so the hundreds of Yen spurs share one
  // allocation. Driven with std::push_heap / std::pop_heap (std::greater) by the
  // search, which is exactly what std::priority_queue does internally.
  std::vector<QueueNode>& heap() { return heap_; }

  // Effective cost from a source. Reads +inf until written this generation.
  double G(int v) const { return Live(v) ? g_[v] : kInfinity_; }
  // Geographic distance along the best path. Valid only for vertices relaxed
  // this generation (the path-reconstruction walk only ever visits those);
  // guarded like G()/Prev() so a stray read of an untouched vertex returns 0
  // rather than a stale value from a previous generation.
  double Geo(int v) const { return Live(v) ? geo_[v] : 0.0; }
  // Predecessor on the best path, or -1 until written this generation.
  int Prev(int v) const { return Live(v) ? prev_[v] : -1; }
  // Inbound heading at v along the best path, in degrees, or -1 until written
  // this generation. Set by Relax to the bearing of the edge the path used to
  // reach v (or the seeded procedure heading for a source). Read by the
  // turn-angle penalty when relaxing v's out-edges and when finishing at a goal.
  double Inbound(int v) const { return Live(v) ? inbound_[v] : -1.0; }
  // Closed is its own generation stamp, so it clears in O(1) with the rest and
  // needs no per-slot byte array (a std::vector<bool> would add bit-masking to
  // the hot pop loop; a stamp compare is a single word comparison).
  bool Closed(int v) const { return closed_stamp_[v] == generation_; }

  // Relax vertex `v`: record cost/distance/predecessor/inbound and stamp it live.
  // `inbound` is the heading the path arrived at v along (the bearing of the
  // edge into v), or -1 for a source with no seeded procedure heading.
  void Relax(int v, double g, double geo, int prev, double inbound) {
    Touch(v);
    g_[v] = g;
    geo_[v] = geo;
    prev_[v] = prev;
    inbound_[v] = inbound;
  }
  void MarkClosed(int v) { closed_stamp_[v] = generation_; }

 private:
  static constexpr double kInfinity_ = std::numeric_limits<double>::infinity();
  bool Live(int v) const { return stamp_[v] == generation_; }
  // Bring a value slot into the current generation, clearing stale state once.
  void Touch(int v) {
    if (stamp_[v] != generation_) {
      stamp_[v] = generation_;
      g_[v] = kInfinity_;
      geo_[v] = 0.0;
      prev_[v] = -1;
      inbound_[v] = -1.0;
    }
  }

  std::vector<double> g_;
  std::vector<double> geo_;
  std::vector<int> prev_;
  std::vector<double> inbound_;         // inbound heading at v along best path (deg), or -1
  std::vector<uint32_t> stamp_;         // value-slot generation tag
  std::vector<uint32_t> closed_stamp_;  // == generation_ => closed this search
  std::vector<QueueNode> heap_;         // open-set backing store, reused across spurs
  uint32_t generation_ = 0;             // bumped per search; 0 = no search run yet
  // generation_ wraps after 2^32 searches; unreachable in practice (a workspace
  // is per-query stack-local and sees at most a few thousand spur searches before
  // it is destroyed), so no wrap handling is needed.
};

// A point on the unit sphere, the Cartesian projection of a Coordinate. Used
// only internally by MultiGoalHeuristic to compute chord-length distances
// without recomputing trig per goal.
struct UnitVec {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// A memoized admissible heuristic for the multi-source/multi-goal search: for a
// vertex it returns the least (great-circle distance to a goal fix + that goal's
// seed cost). The goal set is fixed across a whole Yen run, so h(v) is constant
// per vertex; caching it lets the many spur searches share one table instead of
// recomputing an O(goals) sweep on every pop. Construct once, reuse across
// searches over the SAME graph and goals.
//
// The per-goal distance is a chord length on the unit sphere, not the haversine
// arc: h(v) = kEarthRadiusNm * min_g |unit(v) - unit(g)| + goal.cost. A chord is
// always <= its arc (straight line through the sphere vs. along its surface), so
// it remains an admissible lower bound on the true remaining cost, but it costs
// one sqrt per goal instead of a haversine's atan2/sin/cos -- and because each
// goal's unit vector is precomputed once at construction, a vertex's own unit
// vector (the only trig left) is computed once on the cache miss and reused
// across all goals. The haversine arc is still used where the true distance
// matters (route metrics), never here as a heuristic.
//
// State is function-local to the search (no shared mutable global), so a cache
// owned by a single search or a single Yen invocation stays within the
// concurrency contract: distinct queries build their own instances.
class MultiGoalHeuristic {
 public:
  MultiGoalHeuristic(const NavGraph& graph, const std::vector<SeededEndpoint>& goals);

  // Least remaining cost to finish from `vertex` (memoized).
  double operator()(int vertex) const;

 private:
  const NavGraph& graph_;
  const std::vector<SeededEndpoint>& goals_;
  // Per-goal unit-sphere (x, y, z), precomputed once at construction so the hot
  // path never recomputes goal trig.
  std::vector<UnitVec> goals_xyz_;
  // -1 = not yet computed. Written lazily from the const operator(), so it is
  // NOT thread-safe: this relies on a single instance being used by one search
  // (or one single-threaded Yen run) at a time, per the concurrency note above.
  // Parallelizing the spur searches would share this table across threads and
  // must add synchronization (or switch to a per-thread cache) first.
  mutable std::vector<double> cache_;
};

// Multi-source, multi-goal A*: find the cheapest path that starts at any of
// `sources` (paying its seed cost) and ends at any of `goals` (paying its seed
// cost), over the enroute graph. The returned path's first vertex is the chosen
// source fix and its last vertex the chosen goal fix; distance_nm and cost
// include both seed costs. Returns found=false when no source reaches any goal.
//
// The seed costs are non-negative, so the per-vertex heuristic (great-circle
// distance to the nearest goal fix) stays admissible and the result is optimal.
ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<SeededEndpoint>& goals,
                                   const SearchOptions& options);

// Build a per-vertex seed table: seed[v] is the smallest seed cost among the
// endpoints landing on v, or -1 when v is not an endpoint. Shared by the search
// and by Yen's path re-costing so both agree on which vertices are endpoints.
std::vector<double> BuildSeedTable(const std::vector<SeededEndpoint>& endpoints, int n);

// Build a per-vertex bearing table for the turn-angle constraint:
// bearing[v] is the procedure heading at v (inbound for a source, outbound for a
// goal), or -1 when v is not an endpoint or has no procedure heading. Mirrors
// BuildSeedTable so the search and Yen's re-costing agree on endpoint headings.
std::vector<double> BuildBearingTable(const std::vector<SeededEndpoint>& endpoints, int n);

// Fully reused form for Yen: the caller supplies both the prebuilt goal seed
// table (constant across all spur searches over the same goals) and a workspace
// whose arrays are reused across searches (cleared in O(1) via its generation
// stamp). This is the hot path -- the plain overloads above delegate here after
// building a throwaway seed table and workspace. `goal_seed` must match `graph`
// (size == VertexCount, built by BuildSeedTable from the goal set).
// `source_bearing`/`goal_bearing` (size == VertexCount, from BuildBearingTable)
// supply the procedure headings at the endpoints for the turn-angle penalty;
// pass all--1 tables to disable endpoint turn penalties (e.g. unit-test seams).
ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<double>& goal_seed,
                                   const std::vector<double>& source_bearing,
                                   const std::vector<double>& goal_bearing,
                                   const SearchOptions& options,
                                   const MultiGoalHeuristic& heuristic, SearchWorkspace& ws);

}  // namespace bf
