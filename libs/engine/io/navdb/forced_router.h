// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/graph/astar.h"

namespace bf {

class GraphBuilder;
class NavGraph;

// Via-point (forced) multi-hop routing: resolve tokens to vertices and stitch
// K-shortest hops with CostOfPathMulti re-costing. Stateless static API.
class ForcedRouter {
 public:
  ForcedRouter() = delete;

  // Resolve one forced ("via") point token to a graph vertex. A full
  // "IDENT/REGION" key resolves exactly; a bare ident with several regional
  // matches picks the one adding the least detour to the dep->arr great circle
  // (deterministic and explainable). Airports are rejected (a via point is an
  // enroute fix, and airports are barred as transit nodes anyway). On success,
  // writes the resolved "IDENT/REGION" to `echo`. Returns the vertex, or -1 if no
  // non-airport match exists (the caller reports an unknown-forced-point error).
  static int ResolvePoint(const GraphBuilder& builder, const std::string& token,
                          const Coordinate& from, const Coordinate& to, std::string& echo,
                          bool& is_airport);

  // Stitch a route through an ordered list of forced ("via") vertices. The route
  // is searched in hops -- sources -> F1, Fi -> Fi+1 for each interior pair, then
  // Fn -> goals -- and concatenated. Only the first hop carries the real source
  // seeds and only the last the real goal seeds; interior forced vertices are
  // seeded at 0 so their cost is not double counted at the seams. Every hop
  // honors all constraints and node/edge bans in `options`.
  //
  // Up to `k` whole routes are returned, ordered by total (segment-sum) cost.
  // Each hop is expanded into up to `k` alternatives via K-shortest; the best K
  // end-to-end combinations are then selected by a lazy K-way merge over the
  // Cartesian product. A combination whose stitched path repeats a vertex (a
  // cycle at some seam) is skipped. Returned path distance_nm/cost include both
  // endpoint seeds (CostOfPathMulti), matching non-forced paths for MakeRoute.
  static std::vector<ShortestPath> FindPaths(const NavGraph& graph,
                                             const std::vector<SeededEndpoint>& sources,
                                             const std::vector<SeededEndpoint>& goals,
                                             const std::vector<int>& forced, int k,
                                             const SearchOptions& options);
};

}  // namespace bf
