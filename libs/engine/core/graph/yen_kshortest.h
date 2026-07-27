// SPDX-License-Identifier: LGPL-3.0-or-later
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
// node_filter / edge_filter fields.
std::vector<ShortestPath> FindKShortestPaths(const NavGraph& graph, int start, int goal, int k,
                                             const SearchOptions& base_options);

// Find up to `k` shortest loopless paths over a multi-source / multi-goal search,
// where each candidate may start at a different source fix (paying its seed) and
// end at a different goal fix (paying its seed). This is the procedure-aware form
// used for routing: different candidates can join the network through different
// SID/STAR connection fixes, not just diverge between a single fixed fix pair.
//
// Results are ordered by effective cost ascending (seed + enroute + seed); the
// first equals FindShortestPathMulti. Each path's distance_nm includes both seed
// costs, matching FindShortestPathMulti. `base_options` carries the
// constraints/request and any caller node/edge bans (e.g. "no transit through
// airports"); Yen composes its own bans on top, so callers should not preset the
// node_filter / edge_filter fields with Yen-specific bans.
std::vector<ShortestPath> FindKShortestPathsMulti(const NavGraph& graph,
                                                  const std::vector<SeededEndpoint>& sources,
                                                  const std::vector<SeededEndpoint>& goals, int k,
                                                  const SearchOptions& base_options);

}  // namespace bf
