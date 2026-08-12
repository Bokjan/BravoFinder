// SPDX-License-Identifier: LGPL-3.0-or-later
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/graph/yen_kshortest.h"
#include "io/build/graph_builder.h"
#include "io/navdb/constraint_assembly.h"
#include "io/navdb/endpoint_planner.h"
#include "io/navdb/forced_router.h"
#include "io/navdb/nav_database.h"
#include "io/navdb/route_assembler.h"

namespace bf {

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  const CifpLookup cifp_lookup = [this](const std::string& icao) { return ProceduresFor(icao); };

  Result<EndpointPlan> dep_r =
      EndpointPlanner::Plan(*builder_, request, request.departure, /*departure=*/true, cifp_lookup);
  if (!dep_r) {
    return Result<Routes>::Err(std::move(dep_r).error());
  }
  EndpointPlan dep = std::move(dep_r).value();
  if (dep.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kProcedureNotFound,
                                     "departure airport " + request.departure +
                                         " has no SID matching '" + request.departure_sid + "'"));
  }
  if (dep.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown departure airport: " + request.departure + " (expected an ICAO code, e.g. KLAX)"));
  }
  Result<EndpointPlan> arr_r =
      EndpointPlanner::Plan(*builder_, request, request.arrival, /*departure=*/false, cifp_lookup);
  if (!arr_r) {
    return Result<Routes>::Err(std::move(arr_r).error());
  }
  EndpointPlan arr = std::move(arr_r).value();
  if (arr.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kProcedureNotFound,
                                     "arrival airport " + request.arrival +
                                         " has no STAR matching '" + request.arrival_star + "'"));
  }
  if (arr.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown arrival airport: " + request.arrival + " (expected an ICAO code, e.g. KLAX)"));
  }

  Result<ConstraintBundle> constraints_r = ConstraintAssembly::Build(request, *builder_, mora_);
  if (!constraints_r) {
    return Result<Routes>::Err(std::move(constraints_r).error());
  }
  ConstraintBundle constraints = std::move(constraints_r).value();
  SearchOptions& options = constraints.options;
  const std::vector<int>& avoid_vertices = constraints.avoid_vertices;

  const NavGraph& graph = builder_->graph();
  std::vector<SeededEndpoint> sources =
      EndpointPlanner::ToSearchEndpoints(dep.connections, /*soft_prefer_star=*/false);
  const bool arr_soft_prefer = EndpointPlanner::SoftPreferStarActive(arr);
  std::vector<SeededEndpoint> goals =
      EndpointPlanner::ToSearchEndpoints(arr.connections, arr_soft_prefer);
  // Drop any seeded connection fix the request asks to avoid: it would otherwise
  // slip through as a search start/end, which AvoidConstraint cannot catch.
  if (!avoid_vertices.empty()) {
    auto drop_avoided = [&](std::vector<SeededEndpoint>& eps) {
      eps.erase(std::remove_if(eps.begin(), eps.end(),
                               [&](const SeededEndpoint& e) {
                                 return std::binary_search(avoid_vertices.begin(),
                                                           avoid_vertices.end(), e.vertex);
                               }),
                eps.end());
    };
    drop_avoided(sources);
    drop_avoided(goals);
    if (sources.empty() || goals.empty()) {
      return Result<Routes>::Err(
          Error(ErrorCode::kNoRoute, "no route between endpoints (avoided all connection fixes)"));
    }
  }

  // Resolve forced ("via") points to an ordered vertex list. Disambiguation of a
  // bare ident uses the dep->arr great circle: pick the match adding the least
  // detour. Endpoint coordinates come from the airport vertex when there is one,
  // else from the first seeded connection fix.
  std::vector<int> forced;
  std::vector<std::string> forced_echo;
  if (!request.forced_points.empty()) {
    const int dep_apt = builder_->VertexByAirport(ToUpper(request.departure));
    const int arr_apt = builder_->VertexByAirport(ToUpper(request.arrival));
    const Coordinate dep_coord =
        dep_apt >= 0 ? graph.CoordOf(dep_apt) : graph.CoordOf(sources.front().vertex);
    const Coordinate arr_coord =
        arr_apt >= 0 ? graph.CoordOf(arr_apt) : graph.CoordOf(goals.front().vertex);
    forced.reserve(request.forced_points.size());
    forced_echo.reserve(request.forced_points.size());
    for (const std::string& token : request.forced_points) {
      std::string echo;
      bool is_airport = false;
      const int v =
          ForcedRouter::ResolvePoint(*builder_, token, dep_coord, arr_coord, echo, is_airport);
      if (v < 0) {
        const std::string why =
            is_airport ? "' is an airport, not an enroute waypoint" : "' is not a known waypoint";
        return Result<Routes>::Err(
            Error(ErrorCode::kRouteParseError, "forced point '" + token + why));
      }
      if (std::binary_search(avoid_vertices.begin(), avoid_vertices.end(), v)) {
        return Result<Routes>::Err(Error(ErrorCode::kRouteParseError,
                                         "forced point '" + token + "' is also in the avoid list"));
      }
      forced.push_back(v);
      forced_echo.push_back(echo);
    }
  }

  // Find up to k candidate routes. Unlike the earlier scheme that fixed a single
  // best connection-fix pair and only varied the enroute portion between them,
  // the multi-endpoint Yen lets each candidate join through a different SID/STAR
  // connection fix, so the alternatives can use genuinely different procedures.
  // Both forms report distance_nm with both seed costs already included.
  const int k = std::max(1, request.k);
  std::vector<ShortestPath> paths;
  if (!forced.empty()) {
    // Forced points: search each hop (sources -> F1 -> ... -> Fn -> goals) with
    // K-shortest and merge the best end-to-end combinations. Returns up to k
    // whole routes through the forced points, in cost order.
    paths = ForcedRouter::FindPaths(graph, sources, goals, forced, k, options);
  } else if (k == 1) {
    const ShortestPath best = FindShortestPathMulti(graph, sources, goals, options);
    if (best.found && !best.vertices.empty()) {
      paths.push_back(best);
    }
  } else {
    paths = FindKShortestPathsMulti(graph, sources, goals, k, options);
  }
  if (paths.empty()) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "no route between endpoints"));
  }

  return Result<Routes>::Ok(RouteAssembler::Assemble(*builder_, graph, paths, dep, arr,
                                                     arr_soft_prefer, options, forced_echo));
}

}  // namespace bf
