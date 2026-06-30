#pragma once

#include <vector>

#include "core/graph/astar.h"
#include "core/graph/nav_graph.h"

namespace bf {

// Find up to `k` shortest loopless paths from `start` to `goal` using Yen's
// algorithm on top of the constrained A* search. Results are ordered by
// effective cost ascending; the first is the optimal path. Returns fewer than
// `k` paths when the graph offers fewer alternatives, or an empty vector when
// the goal is unreachable.
//
// `base_options` carries the constraints/request applied to every search; Yen
// adds its own node/edge bans internally, so callers should not set the
// node_blocked / edge_blocked fields.
std::vector<ShortestPath> FindKShortestPaths(const NavGraph& graph, int start, int goal, int k,
                                             const SearchOptions& base_options);

}  // namespace bf
