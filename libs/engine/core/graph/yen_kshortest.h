// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <vector>

#include "core/graph/astar.h"
#include "core/graph/nav_graph.h"

namespace bf {

// Find up to `k` loopless paths from `start` to `goal` using Yen's algorithm on
// top of the constrained A* search. Results are ordered by effective cost
// ascending; the first is the cheapest under the search's cost model. Returns
// fewer than `k` paths when the graph offers fewer alternatives, or an empty
// vector when the goal is unreachable.
//
// With turn_penalty disabled this is classical K-shortest. With turn_penalty
// enabled, A* uses the greedy single-state inbound heading (see TurnPenalty), so
// the K paths are the best under that approximation -- not a guarantee of
// globally penalized-optimal K paths.
//
// `base_options` carries constraints/request and any caller node/edge bans.
// Yen merges its per-spur bans with caller `node_filter.banned` /
// `edge_filter.banned` (airport range on NodeFilter is inherited by copy). Do
// not put Yen-specific bans into base_options yourself.
std::vector<ShortestPath> FindKShortestPaths(const NavGraph& graph, int start, int goal, int k,
                                             const SearchOptions& base_options);

// Multi-source / multi-goal Yen (procedure-aware routing form). Same ordering
// and turn-penalty approximation notes as FindKShortestPaths. The first path
// equals FindShortestPathMulti; distance_nm includes both endpoint seeds.
// Caller bans are merged with Yen's per-spur bans as in FindKShortestPaths.
std::vector<ShortestPath> FindKShortestPathsMulti(const NavGraph& graph,
                                                  const std::vector<SeededEndpoint>& sources,
                                                  const std::vector<SeededEndpoint>& goals, int k,
                                                  const SearchOptions& base_options);

}  // namespace bf
