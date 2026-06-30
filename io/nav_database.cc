#include "io/nav_database.h"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/constraints/altitude_constraints.h"
#include "core/constraints/mora_constraint.h"
#include "core/graph/yen_kshortest.h"
#include "io/graph_builder.h"
#include "io/loaders/xplane/cifp/cifp_parser.h"
#include "io/loaders/xplane/cifp/procedure_connector.h"
#include "io/loaders/xplane/xplane_loader.h"

namespace bf {

namespace {

std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
  return s;
}

// How one endpoint of a query attaches to the enroute graph. An airport with
// procedures contributes several seeded connection fixes; a plain waypoint or a
// DCT-fallback airport contributes one or a few. `airport_icao` is empty for a
// bare waypoint endpoint.
struct EndpointPlan {
  std::vector<Connection> connections;
  std::string airport_icao;  // empty if the endpoint is a plain waypoint
  bool used_procedures = false;
};

// Format a procedure reference as "NAME.TRANSITION" (or just "NAME" when the
// transition is empty / the common segment).
std::string FormatRef(const ProcedureRef& ref) {
  if (ref.transition.empty()) {
    return ref.name;
  }
  return ref.name + "." + ref.transition;
}

// Build a Route (points, legs, route string) from a path of vertex indices. The
// path runs connection-fix to connection-fix; `dep_label`/`arr_label` are the
// airport ICAOs to show as the true endpoints (empty for waypoint endpoints).
Route MakeRoute(const GraphBuilder& builder, const NavGraph& graph, const ShortestPath& path,
                const std::string& dep_label, const std::string& arr_label) {
  Route route;
  route.total_distance_nm = path.distance_nm;

  // The first/last route points are the airports when procedures were used;
  // otherwise the connection fixes themselves are the endpoints.
  if (!dep_label.empty() && !path.vertices.empty()) {
    route.points.push_back(RoutePoint{dep_label, graph.CoordOf(path.vertices.front())});
  }
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder.IdentOf(v).ident, graph.CoordOf(v)});
  }
  if (!arr_label.empty() && !path.vertices.empty()) {
    route.points.push_back(RoutePoint{arr_label, graph.CoordOf(path.vertices.back())});
  }

  // Enroute legs between consecutive on-network fixes.
  for (size_t i = 0; i + 1 < path.vertices.size(); ++i) {
    const int u = path.vertices[i];
    const int w = path.vertices[i + 1];
    std::string via = "DCT";
    double dist = 0.0;
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->to == w) {
        via = builder.AirwayName(e->airway_id);
        dist = e->distance_nm;
        break;
      }
    }
    route.legs.push_back(RouteLeg{builder.IdentOf(u).ident, builder.IdentOf(w).ident, via, dist});
  }

  std::string rs = route.points.empty() ? "" : route.points.front().ident;
  std::string last_via;
  // When an airport label leads, the first hop into the network is via the SID
  // (or DCT); represent it explicitly so the string reads DEP SID FIX ...
  if (!dep_label.empty() && !path.vertices.empty()) {
    rs += " " + builder.IdentOf(path.vertices.front()).ident;
  }
  for (const RouteLeg& leg : route.legs) {
    if (leg.via != last_via) {
      rs += " " + leg.via;
      last_via = leg.via;
    }
    rs += " " + leg.to;
  }
  if (!arr_label.empty()) {
    rs += " " + arr_label;
  }
  route.route_string = rs;
  return route;
}

// Find the connection whose fix is `fix_vertex` and record its procedures on
// the route (SID for departure, STAR for arrival).
void AnnotateProcedures(Route& route, const EndpointPlan& dep, int dep_fix, const EndpointPlan& arr,
                        int arr_fix) {
  for (const Connection& c : dep.connections) {
    if (c.fix_vertex == dep_fix) {
      for (const ProcedureRef& ref : c.procedures) {
        route.sid_options.push_back(FormatRef(ref));
        if (route.sid.empty()) {
          route.sid = ref.name;
          route.dep_runway = ref.runway;
        }
      }
      break;
    }
  }
  for (const Connection& c : arr.connections) {
    if (c.fix_vertex == arr_fix) {
      for (const ProcedureRef& ref : c.procedures) {
        route.star_options.push_back(FormatRef(ref));
        if (route.star.empty()) {
          route.star = ref.name;
          route.arr_runway = ref.runway;
        }
      }
      break;
    }
  }
}

}  // namespace

NavDatabase::NavDatabase() = default;
NavDatabase::~NavDatabase() = default;
NavDatabase::NavDatabase(NavDatabase&&) noexcept = default;
NavDatabase& NavDatabase::operator=(NavDatabase&&) noexcept = default;

Result<NavDatabase> NavDatabase::Open(const std::string& data_dir) {
  Result<NavData> data = XPlaneLoader::Load(data_dir);
  if (!data) {
    return Result<NavDatabase>::Err(std::move(data).error());
  }
  NavDatabase db;
  db.data_dir_ = data_dir;
  db.mora_ = std::move(data.value().mora);
  db.msa_ = std::move(data.value().msa);
  db.builder_ = std::make_unique<GraphBuilder>(data.value());
  return Result<NavDatabase>::Ok(std::move(db));
}

const CifpData* NavDatabase::ProceduresFor(const std::string& icao) const {
  auto it = procedure_cache_.find(icao);
  if (it != procedure_cache_.end()) {
    return it->second.get();  // may be nullptr: a cached "no procedures" result
  }
  Result<CifpData> parsed = CifpParser::Parse(data_dir_ + "/CIFP/" + icao + ".dat");
  std::unique_ptr<CifpData> stored;
  if (parsed) {
    stored = std::make_unique<CifpData>(std::move(parsed).value());
  }
  const CifpData* ptr = stored.get();
  procedure_cache_.emplace(icao, std::move(stored));
  return ptr;
}

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  // Resolve an endpoint into how it attaches to the network. An airport with
  // CIFP procedures connects through them (procedure-first); without CIFP it
  // falls back to DCT links to the nearest on-network waypoints (M1 behavior);
  // a plain waypoint connects as itself.
  auto plan_endpoint = [&](const std::string& name, bool departure) -> EndpointPlan {
    const std::string up = ToUpper(name);
    EndpointPlan plan;
    const int airport = builder_->VertexByAirport(up);
    if (airport >= 0) {
      plan.airport_icao = up;
      const Coordinate apt = builder_->graph().CoordOf(airport);
      const CifpData* cifp = ProceduresFor(up);
      if (cifp != nullptr) {
        const std::string& rwy = departure ? request.departure_runway : request.arrival_runway;
        plan.connections = departure
                               ? ProcedureConnector::BuildDeparture(*cifp, apt, *builder_, rwy)
                               : ProcedureConnector::BuildArrival(*cifp, apt, *builder_, rwy);
        plan.used_procedures = !plan.connections.empty();
      }
      if (plan.connections.empty()) {
        // No usable procedures: fall back to DCT, and clear the airport label
        // so the airport itself is not shown as a procedure endpoint.
        plan.connections = ProcedureConnector::BuildDctFallback(apt, *builder_, 5);
        plan.airport_icao.clear();
      }
      return plan;
    }
    const int wp = builder_->VertexByIdent(up);
    if (wp >= 0) {
      plan.connections.push_back(Connection{wp, 0.0, {}});
    }
    return plan;
  };

  EndpointPlan dep = plan_endpoint(request.departure, /*departure=*/true);
  if (dep.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown departure: " + request.departure));
  }
  EndpointPlan arr = plan_endpoint(request.arrival, /*departure=*/false);
  if (arr.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown arrival: " + request.arrival));
  }

  // Assemble the active constraints from the request.
  AltitudeBandConstraint altitude_band;
  MoraConstraint mora(mora_);
  LevelPreferenceConstraint level_pref;
  SearchOptions options;
  options.request = &request;
  if (request.cruise_fl.has_value()) {
    options.constraints.push_back(&altitude_band);
    options.constraints.push_back(&mora);
  }
  if (request.level != LevelPreference::kNone) {
    options.constraints.push_back(&level_pref);
  }

  const NavGraph& graph = builder_->graph();
  const std::vector<SeededEndpoint> sources = ProcedureConnector::ToEndpoints(dep.connections);
  const std::vector<SeededEndpoint> goals = ProcedureConnector::ToEndpoints(arr.connections);

  // First, find the single best connection-fix pair via multi-source/goal A*.
  const ShortestPath best = FindShortestPathMulti(graph, sources, goals, options);
  if (!best.found || best.vertices.empty()) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "no route between endpoints"));
  }
  const int dep_fix = best.vertices.front();
  const int arr_fix = best.vertices.back();

  // For k>1, expand alternatives between the chosen connection fixes with Yen.
  // The seed distances are constant offsets for a fixed fix pair, so ranking by
  // enroute cost preserves the overall ordering.
  const int k = std::max(1, request.k);
  std::vector<ShortestPath> paths;
  if (k == 1 || dep_fix == arr_fix) {
    paths.push_back(best);
  } else {
    paths = FindKShortestPaths(graph, dep_fix, arr_fix, k, options);
    if (paths.empty()) {
      paths.push_back(best);
    }
  }

  // Seed distances to add back when Yen reported only the enroute portion.
  double dep_seed = 0.0;
  for (const SeededEndpoint& s : sources) {
    if (s.vertex == dep_fix) {
      dep_seed = s.cost;
      break;
    }
  }
  double arr_seed = 0.0;
  for (const SeededEndpoint& g : goals) {
    if (g.vertex == arr_fix) {
      arr_seed = g.cost;
      break;
    }
  }

  Routes routes;
  routes.reserve(paths.size());
  for (const ShortestPath& p : paths) {
    ShortestPath adjusted = p;
    // The multi-source result already includes both seeds; Yen paths do not.
    if (!(k == 1 || dep_fix == arr_fix)) {
      adjusted.distance_nm += dep_seed + arr_seed;
    }
    Route route = MakeRoute(*builder_, graph, adjusted, dep.airport_icao, arr.airport_icao);
    AnnotateProcedures(route, dep, dep_fix, arr, arr_fix);
    routes.push_back(std::move(route));
  }
  return Result<Routes>::Ok(std::move(routes));
}

std::vector<MsaSector> NavDatabase::MsaForAirport(const std::string& icao) const {
  const std::string up = ToUpper(icao);
  std::vector<MsaSector> out;
  for (const MsaSector& s : msa_) {
    if (s.airport_icao == up) {
      out.push_back(s);
    }
  }
  return out;
}

}  // namespace bf
