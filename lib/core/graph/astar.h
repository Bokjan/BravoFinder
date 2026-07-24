#pragma once

#include <cstdint>
#include <functional>
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

// Optional inputs that shape a search: routing constraints and the bans Yen's
// algorithm uses to carve out alternative paths. All fields are optional; an
// empty SearchOptions reproduces a plain shortest-path search.
struct SearchOptions {
  // Constraints applied to every edge (hard filter + soft penalty). The request
  // they are evaluated against must be supplied when constraints are present.
  std::vector<const Constraint*> constraints;
  const RouteRequest* request = nullptr;

  // Yen support: vertices and directed edges that must not be used.
  std::function<bool(int)> node_blocked;
  std::function<bool(int from, int to)> edge_blocked;
};

// Reusable per-vertex A* scratch, defined below; forward-declared here so the
// hot-path FindShortestPath overload can take it by reference.
class SearchWorkspace;

// Among the parallel edges from `from` to `to`, return the one the search would
// have traversed: the cheapest ALLOWED edge by effective cost (distance_nm + soft
// penalties), evaluating `options.constraints` exactly as A* relaxation and Yen's
// path re-costing do. Returns nullptr if there is no such edge (none exists, or
// every parallel edge is blocked). Because every constraint is a deterministic
// function of the edge, this reproduces the search's choice, so a route's leg
// labels (airway, distance) match the path the search actually cost -- not the
// merely shortest-by-distance parallel edge.
const GraphEdge* SelectEdge(const NavGraph& graph, int from, int to, const SearchOptions& options);

// Find the shortest path from `start` to `goal` using A* with an admissible
// great-circle heuristic. Soft penalties only add cost, so the geographic
// heuristic remains a lower bound and the result is optimal under the effective
// (penalized) cost. Returns found=false when no path exists.
ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal,
                              const SearchOptions& options);

// Reused form for Yen: the caller supplies a workspace whose per-vertex arrays
// are allocated once and cleared in O(1) between searches via a generation stamp.
// Yen's single-source variant spurs the search hundreds of times over the same
// graph; without a shared workspace each spur re-ran the O(V) Reset (five arrays
// sized to ~270k vertices), which is exactly the cost the stamp design removes.
// The workspace is reset to a fresh generation on entry, so callers may pass a
// dirty one; it must not be shared across concurrent searches.
ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal,
                              const SearchOptions& options, SearchWorkspace& ws);

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

  // Effective cost from a source. Reads +inf until written this generation.
  double G(int v) const { return Live(v) ? g_[v] : kInfinity_; }
  // Geographic distance along the best path. Valid only for vertices relaxed
  // this generation (the path-reconstruction walk only ever visits those);
  // guarded like G()/Prev() so a stray read of an untouched vertex returns 0
  // rather than a stale value from a previous generation.
  double Geo(int v) const { return Live(v) ? geo_[v] : 0.0; }
  // Predecessor on the best path, or -1 until written this generation.
  int Prev(int v) const { return Live(v) ? prev_[v] : -1; }
  // Closed is its own generation stamp, so it clears in O(1) with the rest and
  // needs no per-slot byte array (a std::vector<bool> would add bit-masking to
  // the hot pop loop; a stamp compare is a single word comparison).
  bool Closed(int v) const { return closed_stamp_[v] == generation_; }

  // Relax vertex `v`: record cost/distance/predecessor and stamp it live.
  void Relax(int v, double g, double geo, int prev) {
    Touch(v);
    g_[v] = g;
    geo_[v] = geo;
    prev_[v] = prev;
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
    }
  }

  std::vector<double> g_;
  std::vector<double> geo_;
  std::vector<int> prev_;
  std::vector<uint32_t> stamp_;         // value-slot generation tag
  std::vector<uint32_t> closed_stamp_;  // == generation_ => closed this search
  uint32_t generation_ = 0;             // bumped per search; 0 = no search run yet
  // generation_ wraps after 2^32 searches; unreachable in practice (a workspace
  // is per-query stack-local and sees at most a few thousand spur searches before
  // it is destroyed), so no wrap handling is needed.
};

// A memoized admissible heuristic for the multi-source/multi-goal search: for a
// vertex it returns the least (great-circle distance to a goal fix + that goal's
// seed cost). The goal set is fixed across a whole Yen run, so h(v) is constant
// per vertex; caching it lets the many spur searches share one table instead of
// recomputing an O(goals) haversine sweep on every pop. Construct once, reuse
// across searches over the SAME graph and goals.
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

// Overload taking a caller-owned, memoized heuristic so repeated searches with
// the same graph and goals (Yen's spur searches) share one cache. `heuristic`
// must have been built for the same `graph` and `goals`.
ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<SeededEndpoint>& goals,
                                   const SearchOptions& options,
                                   const MultiGoalHeuristic& heuristic);

// Build a per-vertex seed table: seed[v] is the smallest seed cost among the
// endpoints landing on v, or -1 when v is not an endpoint. Shared by the search
// and by Yen's path re-costing so both agree on which vertices are endpoints.
std::vector<double> BuildSeedTable(const std::vector<SeededEndpoint>& endpoints, int n);

// Fully reused form for Yen: the caller supplies both the prebuilt goal seed
// table (constant across all spur searches over the same goals) and a workspace
// whose arrays are reused across searches (cleared in O(1) via its generation
// stamp). This is the hot path -- the plain overloads above delegate here after
// building a throwaway seed table and workspace. `goal_seed` must match `graph`
// (size == VertexCount, built by BuildSeedTable from the goal set).
ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<double>& goal_seed,
                                   const SearchOptions& options,
                                   const MultiGoalHeuristic& heuristic, SearchWorkspace& ws);

}  // namespace bf
