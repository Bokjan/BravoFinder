// SPDX-License-Identifier: LGPL-3.0-or-later
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/domain/nav_tokens.h"
#include "core/graph/yen_kshortest.h"
#include "core/routing/route_string.h"
#include "io/build/graph_builder.h"
#include "io/navdb/constraint_assembly.h"
#include "io/navdb/endpoint_planner.h"
#include "io/navdb/forced_router.h"
#include "io/navdb/nav_database.h"

namespace bf {

namespace {

// Format a procedure reference as "NAME.TRANSITION" (or just "NAME" when the
// transition is empty / the common segment).
std::string FormatRef(const ProcedureRef& ref) {
  if (ref.transition.empty()) {
    return ref.name;
  }
  return ref.name + "." + ref.transition;
}

const Connection* FindConnection(const EndpointPlan& plan, int fix_vertex) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex == fix_vertex) {
      return &c;
    }
  }
  return nullptr;
}

Route MakeRoute(const GraphBuilder& builder, const NavGraph& graph, const ShortestPath& path,
                const std::string& dep_label, const std::string& arr_label, const std::string& sid,
                const std::string& star, double dep_seed, double arr_seed,
                const SearchOptions& options, int dep_splice_vertex = kNoVertex,
                double dep_splice_leg_nm = 0.0, int arr_splice_vertex = kNoVertex,
                double arr_splice_leg_nm = 0.0) {
  Route route;
  route.total_distance_nm = path.distance_nm;
  if (path.vertices.empty()) {
    return route;
  }
  const std::string dep_fix_id = builder.IdentOf(path.vertices.front()).ident;
  const std::string arr_fix_id = builder.IdentOf(path.vertices.back()).ident;
  const bool dep_splice = dep_splice_vertex >= 0 && !dep_label.empty();
  const bool arr_splice = arr_splice_vertex >= 0 && !arr_label.empty();
  const double dep_body_nm = dep_splice ? std::max(0.0, dep_seed - dep_splice_leg_nm) : dep_seed;
  const double arr_body_nm = arr_splice ? std::max(0.0, arr_seed - arr_splice_leg_nm) : arr_seed;
  const std::string dep_exit_id =
      dep_splice ? builder.IdentOf(dep_splice_vertex).ident : std::string{};
  const std::string arr_entry_id =
      arr_splice ? builder.IdentOf(arr_splice_vertex).ident : std::string{};

  // Points: optional departure airport, optional SID exit (splice), the
  // connection fixes along the path, optional STAR entry (splice), then the
  // optional arrival airport.
  if (!dep_label.empty()) {
    const int dep_apt = builder.VertexByAirport(ToUpper(dep_label));
    const Coordinate dep_coord =
        dep_apt >= 0 ? graph.CoordOf(dep_apt) : graph.CoordOf(path.vertices.front());
    route.points.push_back(RoutePoint{dep_label, dep_coord});
    if (dep_splice) {
      route.points.push_back(RoutePoint{dep_exit_id, graph.CoordOf(dep_splice_vertex)});
    }
  }
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder.IdentOf(v).ident, graph.CoordOf(v)});
  }
  if (!arr_label.empty()) {
    if (arr_splice) {
      route.points.push_back(RoutePoint{arr_entry_id, graph.CoordOf(arr_splice_vertex)});
    }
    const int arr_apt = builder.VertexByAirport(ToUpper(arr_label));
    const Coordinate arr_coord =
        arr_apt >= 0 ? graph.CoordOf(arr_apt) : graph.CoordOf(path.vertices.back());
    route.points.push_back(RoutePoint{arr_label, arr_coord});
  }

  // Leading procedure leg(s): airport -> [exit DCT] -> first connection fix.
  if (!dep_label.empty()) {
    if (dep_splice) {
      route.legs.push_back(
          RouteLeg{dep_label, dep_exit_id, std::string(kSidToken), dep_body_nm, {}});
      route.legs.push_back(
          RouteLeg{dep_exit_id, dep_fix_id, std::string(kDctToken), dep_splice_leg_nm, {}});
    } else {
      route.legs.push_back(RouteLeg{dep_label,
                                    dep_fix_id,
                                    sid.empty() ? std::string(kDctToken) : std::string(kSidToken),
                                    dep_seed,
                                    {}});
    }
  }
  // Enroute legs between consecutive on-network fixes.
  for (size_t i = 0; i + 1 < path.vertices.size(); ++i) {
    const int u = path.vertices[i];
    const int w = path.vertices[i + 1];
    const GraphEdge* e = SelectEdge(graph, u, w, options);
    std::string via(kDctToken);
    double dist = 0.0;
    if (e != nullptr) {
      via = builder.AirwayName(e->airway_id);
      dist = e->distance_nm;
    }
    route.legs.push_back(
        RouteLeg{builder.IdentOf(u).ident, builder.IdentOf(w).ident, via, dist, {}});
  }
  // Trailing procedure leg(s): last connection fix -> [entry STAR] -> airport.
  if (!arr_label.empty()) {
    if (arr_splice) {
      route.legs.push_back(
          RouteLeg{arr_fix_id, arr_entry_id, std::string(kDctToken), arr_splice_leg_nm, {}});
      route.legs.push_back(
          RouteLeg{arr_entry_id, arr_label, std::string(kStarToken), arr_body_nm, {}});
    } else {
      route.legs.push_back(RouteLeg{arr_fix_id,
                                    arr_label,
                                    star.empty() ? std::string(kDctToken) : std::string(kStarToken),
                                    arr_seed,
                                    {}});
    }
  }

  // Phase split (D9): dep/arr are literal SID/STAR leg distances; DCT splice
  // falls into enroute. Search seeds still priced the full |F→gate|+body.
  route.dep_distance_nm = dep_label.empty() ? 0.0 : dep_body_nm;
  route.arr_distance_nm = arr_label.empty() ? 0.0 : arr_body_nm;
  route.enroute_distance_nm =
      std::max(0.0, route.total_distance_nm - route.dep_distance_nm - route.arr_distance_nm);

  const std::string first_point = route.points.empty() ? "" : route.points.front().ident;
  route.route_string = BuildRouteString(first_point, route.legs);
  return route;
}

// Pick the primary procedure (name + runway) and all interchangeable options
// for the connection fix `fix_vertex` within `plan`. Returns the chosen name in
// `name`/`runway` and every "NAME.TRANSITION" sharing the fix in `options`.
void SelectProcedures(const EndpointPlan& plan, int fix_vertex, std::string& name,
                      std::string& runway, std::vector<std::string>& options) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex != fix_vertex) {
      continue;
    }
    for (const ProcedureRef& ref : c.procedures) {
      // Approach refs are not STARs — handled by SelectApproachProcedures.
      if (ref.type == ProcedureType::kApproach) {
        continue;
      }
      options.push_back(FormatRef(ref));
      if (name.empty()) {
        name = ref.name;
        runway = ref.runway;
      }
    }
    break;
  }
}

// Fill approach_* metadata for a terminal-transition arrival at `fix_vertex`.
void SelectApproachProcedures(const EndpointPlan& plan, int fix_vertex, std::string& approach,
                              std::string& approach_iaf, double& approach_bearing,
                              std::string& runway, std::vector<std::string>& options) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex != fix_vertex) {
      continue;
    }
    approach_bearing = c.approach_bearing;
    for (const ProcedureRef& ref : c.procedures) {
      if (ref.type != ProcedureType::kApproach) {
        continue;
      }
      options.push_back(FormatRef(ref));
      if (approach.empty()) {
        approach = FormatRef(ref);
        approach_iaf = ref.iaf;
        runway = ref.runway;
      }
    }
    break;
  }
}

}  // namespace

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

  // Look up a connection fix's seed cost among an endpoint's seeded fixes.
  // Geographic seed for MakeRoute / phase split — never the soft-prefer bump.
  auto raw_seed_of = [](const EndpointPlan& plan, int vertex) {
    const Connection* c = FindConnection(plan, vertex);
    return c != nullptr ? c->seed_distance_nm : 0.0;
  };

  // Classify how an endpoint attached to the network. Departure plans stay
  // homogeneous (SID or DCT). Arrival may mix STAR splice with approach in one
  // pool (#30) — those paths must be classified from the winning Connection's
  // procedures.front().type, not from plan-level used_* flags.
  auto connection_kind = [](const EndpointPlan& plan) {
    if (plan.used_approach) {
      return ConnectionKind::kTerminalTransition;
    }
    if (plan.used_procedures) {
      return ConnectionKind::kProcedure;
    }
    if (plan.has_procedures) {
      return ConnectionKind::kRadarVectors;
    }
    return ConnectionKind::kDirect;
  };
  auto arrival_kind_for_fix = [&](int arr_fix) {
    const Connection* c = FindConnection(arr, arr_fix);
    if (c != nullptr && !c->procedures.empty()) {
      if (c->procedures.front().type == ProcedureType::kApproach) {
        return ConnectionKind::kTerminalTransition;
      }
      return ConnectionKind::kProcedure;
    }
    return connection_kind(arr);
  };
  const ConnectionKind dep_kind = connection_kind(dep);

  Routes routes;
  routes.reserve(paths.size());
  for (const ShortestPath& p : paths) {
    if (p.vertices.empty()) {
      continue;
    }
    const int dep_fix = p.vertices.front();
    const int arr_fix = p.vertices.back();
    const double dep_seed = raw_seed_of(dep, dep_fix);
    const double arr_seed = raw_seed_of(arr, arr_fix);
    const Connection* dep_conn = FindConnection(dep, dep_fix);
    const Connection* arr_conn = FindConnection(arr, arr_fix);
    const int dep_splice_v = dep_conn != nullptr ? dep_conn->splice_vertex : kNoVertex;
    const double dep_splice_nm = dep_conn != nullptr ? dep_conn->splice_leg_nm : 0.0;
    const int arr_splice_v = arr_conn != nullptr ? arr_conn->splice_vertex : kNoVertex;
    const double arr_splice_nm = arr_conn != nullptr ? arr_conn->splice_leg_nm : 0.0;

    // Procedure selection depends on the candidate's own fix pair, which may
    // differ across candidates, so resolve it per path.
    std::string sid_name;
    std::string dep_rwy;
    std::vector<std::string> sid_options;
    SelectProcedures(dep, dep_fix, sid_name, dep_rwy, sid_options);
    std::string star_name;
    std::string arr_rwy;
    std::vector<std::string> star_options;
    std::string approach;
    std::string approach_iaf;
    double approach_bearing = -1.0;
    std::vector<std::string> approach_options;
    const ConnectionKind arr_kind = arrival_kind_for_fix(arr_fix);
    if (arr_kind == ConnectionKind::kTerminalTransition) {
      SelectApproachProcedures(arr, arr_fix, approach, approach_iaf, approach_bearing, arr_rwy,
                               approach_options);
    } else {
      SelectProcedures(arr, arr_fix, star_name, arr_rwy, star_options);
    }

    Route route = MakeRoute(*builder_, graph, p, dep.airport_icao, arr.airport_icao, sid_name,
                            star_name, dep_seed, arr_seed, options, dep_splice_v, dep_splice_nm,
                            arr_splice_v, arr_splice_nm);
    // A* folds SeededEndpoint.cost into distance_nm; strip the soft-prefer bump
    // so reported totals stay geographic when an approach goal won the mixed pool.
    if (arr_soft_prefer && arr_kind == ConnectionKind::kTerminalTransition) {
      route.total_distance_nm =
          std::max(0.0, route.total_distance_nm - EndpointPlanner::kProcedurePreferNm);
      route.enroute_distance_nm =
          std::max(0.0, route.total_distance_nm - route.dep_distance_nm - route.arr_distance_nm);
    }
    route.sid = sid_name;
    route.dep_runway = dep_rwy;
    route.sid_options = sid_options;
    route.star = star_name;
    route.arr_runway = arr_rwy;
    route.star_options = star_options;
    route.dep_connection = dep_kind;
    route.arr_connection = arr_kind;
    if (arr_kind == ConnectionKind::kTerminalTransition) {
      route.terminal_transition = true;
      route.approach = std::move(approach);
      route.approach_iaf = std::move(approach_iaf);
      route.approach_bearing = approach_bearing;
      route.approach_options = std::move(approach_options);
    }
    route.forced_points = forced_echo;
    routes.push_back(std::move(route));
  }
  return Result<Routes>::Ok(std::move(routes));
}

}  // namespace bf
