// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/navdb/route_assembler.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/domain/nav_tokens.h"
#include "core/routing/route_string.h"
#include "io/build/graph_builder.h"
#include "io/build/procedure_connector.h"

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

ConnectionKind ConnectionKindOf(const EndpointPlan& plan) {
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
}

ConnectionKind ArrivalKindForFix(const EndpointPlan& arr, int arr_fix) {
  const Connection* c = FindConnection(arr, arr_fix);
  if (c != nullptr && !c->procedures.empty()) {
    if (c->procedures.front().type == ProcedureType::kApproach) {
      return ConnectionKind::kTerminalTransition;
    }
    return ConnectionKind::kProcedure;
  }
  return ConnectionKindOf(arr);
}

}  // namespace

std::vector<Route> RouteAssembler::Assemble(const GraphBuilder& builder, const NavGraph& graph,
                                            const std::vector<ShortestPath>& paths,
                                            const EndpointPlan& dep, const EndpointPlan& arr,
                                            bool arr_soft_prefer, const SearchOptions& options,
                                            const std::vector<std::string>& forced_echo) {
  // Geographic seed for MakeRoute / phase split — never the soft-prefer bump.
  auto raw_seed_of = [](const EndpointPlan& plan, int vertex) {
    const Connection* c = FindConnection(plan, vertex);
    return c != nullptr ? c->seed_distance_nm : 0.0;
  };

  const ConnectionKind dep_kind = ConnectionKindOf(dep);

  std::vector<Route> routes;
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
    const ConnectionKind arr_kind = ArrivalKindForFix(arr, arr_fix);
    if (arr_kind == ConnectionKind::kTerminalTransition) {
      SelectApproachProcedures(arr, arr_fix, approach, approach_iaf, approach_bearing, arr_rwy,
                               approach_options);
    } else {
      SelectProcedures(arr, arr_fix, star_name, arr_rwy, star_options);
    }

    Route route = MakeRoute(builder, graph, p, dep.airport_icao, arr.airport_icao, sid_name,
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
  return routes;
}

}  // namespace bf
