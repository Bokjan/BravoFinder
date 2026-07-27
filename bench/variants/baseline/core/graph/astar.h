// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cstdint>
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
// Pack a directed edge (from, to) into one 64-bit key. Vertex ids are small
// non-negative ints, so a flat integer key lets a banned-edge set be a sorted
// vector with binary_search -- no std::pair ordering, no hashing, no
// std::function type erasure on the A* hot loop.
inline int64_t EdgeKey(int from, int to) {
  return (static_cast<int64_t>(from) << 32) | static_cast<uint32_t>(to);
}

// A node block filter for the A* hot loop. Combines an optional airport-range
// block (airports occupy the contiguous tail [airport_first, airport_last) of
// the vertex range, so this is a two-compare range check) with an optional
// sorted banned-vertex set (Yen's per-spur root-node ban). An empty NodeFilter
// blocks nothing. Blocks() is straight-line code (range check + binary_search),
// inlinable with no type-erased call.
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
// EdgeFilter blocks nothing.
struct EdgeFilter {
  const std::vector<int64_t>* banned = nullptr;  // sorted ascending, or nullptr

  bool Blocks(int from, int to) const {
    return banned != nullptr &&
           std::binary_search(banned->begin(), banned->end(), EdgeKey(from, to));
  }
};

struct SearchOptions {
  // Constraints applied to every edge (hard filter + soft penalty). The request
  // they are evaluated against must be supplied when constraints are present.
  std::vector<const Constraint*> constraints;
  const RouteRequest* request = nullptr;

  // Yen support: vertices and directed edges that must not be used.
  NodeFilter node_filter;
  EdgeFilter edge_filter;
};

// Among the parallel edges from `from` to `to`, return the one the search would
// have traversed: the cheapest ALLOWED edge by effective cost (distance_nm +
// soft penalties), evaluating `options.constraints` exactly as A* relaxation
// does. Returns nullptr if there is no such edge (none exists, or every parallel
// edge is blocked). Because every constraint is a deterministic function of the
// edge, this reproduces the search's choice, so a route's leg labels match the
// path the search actually cost. `out_cost` receives the selected edge's
// effective cost when non-null.
const GraphEdge* SelectEdge(const NavGraph& graph, int from, int to, const SearchOptions& options,
                            double* out_cost = nullptr);

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
