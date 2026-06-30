#pragma once

#include <string>
#include <vector>

#include "core/domain/coordinate.h"

namespace bf {

// A single leg of a computed route: a segment from one point to the next via a
// named airway (or "DCT" for a direct leg).
struct RouteLeg {
  std::string from;
  std::string to;
  std::string via;  // airway name, or "DCT"
  double distance_nm = 0.0;
};

// A point along a computed route, for display / export.
struct RoutePoint {
  std::string ident;
  Coordinate coord;
};

// A computed route between two endpoints.
struct Route {
  std::vector<RoutePoint> points;
  std::vector<RouteLeg> legs;
  double total_distance_nm = 0.0;
  std::string route_string;  // compact "DEP DCT WPT AWY ... ARR" form
};

}  // namespace bf
