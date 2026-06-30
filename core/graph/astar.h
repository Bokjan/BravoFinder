#pragma once

#include <vector>

#include "core/graph/nav_graph.h"

namespace bf {

// The outcome of an A* search.
struct ShortestPath {
  std::vector<int> vertices;  // start..goal inclusive; empty if unreachable
  double distance_nm = 0.0;   // total path length
  bool found = false;
};

// Find the shortest path from `start` to `goal` in `graph` using A* with a
// great-circle-distance heuristic. The heuristic is admissible (it never
// overestimates the remaining distance), so the result is optimal. Returns a
// ShortestPath with found=false when no path exists.
ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal);

}  // namespace bf
