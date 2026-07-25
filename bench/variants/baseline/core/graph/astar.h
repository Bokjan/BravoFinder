// SPDX-License-Identifier: MIT
#pragma once

#include <functional>
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

// Find the shortest path from `start` to `goal` using A* with an admissible
// great-circle heuristic. Soft penalties only add cost, so the geographic
// heuristic remains a lower bound and the result is optimal under the effective
// (penalized) cost. Returns found=false when no path exists.
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

}  // namespace bf
