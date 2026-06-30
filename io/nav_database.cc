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
#include "io/loaders/xplane/xplane_loader.h"

namespace bf {

namespace {

std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
  return s;
}

// Build a Route (points, legs, route string) from a path of vertex indices.
Route MakeRoute(const GraphBuilder& builder, const NavGraph& graph, const ShortestPath& path) {
  Route route;
  route.total_distance_nm = path.distance_nm;
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder.IdentOf(v).ident, graph.CoordOf(v)});
  }
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
  for (const RouteLeg& leg : route.legs) {
    if (leg.via != last_via) {
      rs += " " + leg.via;
      last_via = leg.via;
    }
    rs += " " + leg.to;
  }
  route.route_string = rs;
  return route;
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
  db.mora_ = std::move(data.value().mora);
  db.msa_ = std::move(data.value().msa);
  db.builder_ = std::make_unique<GraphBuilder>(data.value());
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  auto resolve = [&](const std::string& name) {
    const std::string up = ToUpper(name);
    int v = builder_->VertexByAirport(up);
    if (v < 0) {
      v = builder_->VertexByIdent(up);
    }
    return v;
  };

  const int start = resolve(request.departure);
  if (start < 0) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown departure: " + request.departure));
  }
  const int goal = resolve(request.arrival);
  if (goal < 0) {
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
  const int k = std::max(1, request.k);
  std::vector<ShortestPath> paths = FindKShortestPaths(graph, start, goal, k, options);
  if (paths.empty()) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "no route between endpoints"));
  }

  Routes routes;
  routes.reserve(paths.size());
  for (const ShortestPath& p : paths) {
    routes.push_back(MakeRoute(*builder_, graph, p));
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
