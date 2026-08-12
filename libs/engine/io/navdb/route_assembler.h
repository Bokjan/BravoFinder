// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/graph/astar.h"
#include "core/routing/route.h"
#include "io/navdb/endpoint_planner.h"

namespace bf {

class GraphBuilder;
class NavGraph;

// Turns ShortestPath results (+ endpoint plans) into filed Routes: procedure
// metadata, splice legs, soft-prefer distance strip, forced-point echo.
// Stateless; all entry points are static.
class RouteAssembler {
 public:
  RouteAssembler() = delete;

  // Assemble one Route per non-empty path. `arr_soft_prefer` must match the flag
  // used when seeding arrival endpoints (strips kProcedurePreferNm from reported
  // totals when an approach goal won a mixed STAR∪approach pool).
  static std::vector<Route> Assemble(const GraphBuilder& builder, const NavGraph& graph,
                                     const std::vector<ShortestPath>& paths,
                                     const EndpointPlan& dep, const EndpointPlan& arr,
                                     bool arr_soft_prefer, const SearchOptions& options,
                                     const std::vector<std::string>& forced_echo);
};

}  // namespace bf
