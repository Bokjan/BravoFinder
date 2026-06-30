#pragma once

#include <memory>
#include <string>

#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"

namespace bf {

class GraphBuilder;

// The top-level navigation database: owns the loaded data and the route graph,
// and answers route queries. Self-contained with no global/static state, so
// multiple instances (e.g. different AIRAC cycles) can coexist safely.
class NavDatabase {
 public:
  NavDatabase();
  ~NavDatabase();
  NavDatabase(NavDatabase&&) noexcept;
  NavDatabase& operator=(NavDatabase&&) noexcept;

  // Load X-Plane data from `data_dir` and build the route graph. Returns the
  // ready database or an Error.
  static Result<NavDatabase> Open(const std::string& data_dir);

  // Find a route for `request`. Endpoints are resolved as airport ICAO first,
  // then as a waypoint ident. Case-insensitive.
  Result<Route> FindRoute(const RouteRequest& request) const;

 private:
  std::unique_ptr<GraphBuilder> builder_;
};

}  // namespace bf
