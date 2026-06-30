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

  // Terminal procedures used to connect the airports to the enroute network.
  // Empty when an endpoint is a plain waypoint or fell back to a DCT link.
  std::string sid;         // departure SID name, e.g. "DEEZZ5"
  std::string star;        // arrival STAR name, e.g. "CAMRN5"
  std::string dep_runway;  // departure runway if known, e.g. "RW31L"
  std::string arr_runway;  // arrival runway if known

  // All SID/STAR(+runway) combinations that share the chosen connection fix and
  // are therefore interchangeable for this route, formatted "NAME.TRANSITION".
  // The route was computed once for the shared fix rather than per procedure.
  std::vector<std::string> sid_options;
  std::vector<std::string> star_options;
};

}  // namespace bf
