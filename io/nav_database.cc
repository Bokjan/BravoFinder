#include "io/nav_database.h"

#include <algorithm>
#include <cctype>
#include <string>
#include <utility>

#include "core/graph/astar.h"
#include "io/graph_builder.h"
#include "io/loaders/xplane/xplane_loader.h"

namespace bf {

namespace {

std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
  return s;
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
  db.builder_ = std::make_unique<GraphBuilder>(data.value());
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<Route> NavDatabase::FindRoute(const RouteRequest& request) const {
  if (!builder_) {
    return Result<Route>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  // Resolve an endpoint: airport ICAO first, then waypoint ident.
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
    return Result<Route>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown departure: " + request.departure));
  }
  const int goal = resolve(request.arrival);
  if (goal < 0) {
    return Result<Route>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown arrival: " + request.arrival));
  }

  const NavGraph& graph = builder_->graph();
  ShortestPath path = FindShortestPath(graph, start, goal);
  if (!path.found) {
    return Result<Route>::Err(Error(ErrorCode::kNoRoute, "no route between endpoints"));
  }

  // Build the Route: points, legs, and a compact route string.
  Route route;
  route.total_distance_nm = path.distance_nm;
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder_->IdentOf(v).ident, graph.CoordOf(v)});
  }
  for (size_t i = 0; i + 1 < path.vertices.size(); ++i) {
    const int u = path.vertices[i];
    const int w = path.vertices[i + 1];
    // Find the edge u->w that was used, to recover airway name and distance.
    std::string via = "DCT";
    double dist = 0.0;
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->to == w) {
        via = builder_->AirwayName(e->airway_id);
        dist = e->distance_nm;
        break;
      }
    }
    route.legs.push_back(
        RouteLeg{builder_->IdentOf(u).ident, builder_->IdentOf(w).ident, via, dist});
  }

  // Compact route string: collapse consecutive legs on the same airway.
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

  return Result<Route>::Ok(std::move(route));
}

}  // namespace bf
