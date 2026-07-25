// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "core/routing/route.h"
#include "core/routing/route_metrics.h"

namespace bf {

// Serialize a Route as a JSON object into any RapidJSON-style Writer (the
// Writer streams directly to its buffer -- no intermediate DOM -- and escapes
// strings correctly). This is a header-only template so it adds no dependency
// to bf3; callers link RapidJSON themselves and pass a Writer.
//
// The template only names Writer's duck-typed methods (StartObject, Key,
// String, Double, StartArray, ...), so it works with rapidjson::Writer,
// PrettyWriter, or any compatible sink. Sizes are passed as unsigned, matching
// rapidjson::SizeType, so no RapidJSON header is needed here.
template <class Writer>
void WriteRouteJson(Writer& writer, const Route& route) {
  auto str = [&](const std::string& s) {
    writer.String(s.c_str(), static_cast<unsigned>(s.size()));
  };
  auto key_str = [&](const char* k, const std::string& s) {
    writer.Key(k);
    str(s);
  };
  auto key_str_array = [&](const char* k, const std::vector<std::string>& items) {
    writer.Key(k);
    writer.StartArray();
    for (const std::string& s : items) {
      str(s);
    }
    writer.EndArray();
  };

  writer.StartObject();
  key_str("route", route.route_string);
  writer.Key("total_distance_nm");
  writer.Double(route.total_distance_nm);
  writer.Key("dep_distance_nm");
  writer.Double(route.dep_distance_nm);
  writer.Key("enroute_distance_nm");
  writer.Double(route.enroute_distance_nm);
  writer.Key("arr_distance_nm");
  writer.Double(route.arr_distance_nm);
  key_str("sid", route.sid);
  key_str("dep_runway", route.dep_runway);
  key_str_array("sid_options", route.sid_options);
  key_str("star", route.star);
  key_str("arr_runway", route.arr_runway);
  key_str_array("star_options", route.star_options);
  if (!route.forced_points.empty()) {
    key_str_array("forced_points", route.forced_points);
  }
  writer.Key("dep_connection");
  writer.String(ToString(route.dep_connection));
  writer.Key("arr_connection");
  writer.String(ToString(route.arr_connection));

  // Ordered points along the route, each carrying its ident and coordinate.
  // points is parallel to legs with one extra entry (N points, N-1 legs): the
  // destination of leg i is points[i+1].
  writer.Key("points");
  writer.StartArray();
  for (const RoutePoint& point : route.points) {
    writer.StartObject();
    key_str("ident", point.ident);
    writer.Key("lat");
    writer.Double(point.coord.latitude);
    writer.Key("lon");
    writer.Double(point.coord.longitude);
    writer.EndObject();
  }
  writer.EndArray();

  // Running distance after each leg (parallel to legs), so consumers do not have
  // to accumulate distance_nm themselves. The last entry equals total_distance_nm.
  const std::vector<double> cumulative = CumulativeDistances(route.legs);
  writer.Key("legs");
  writer.StartArray();
  for (size_t i = 0; i < route.legs.size(); ++i) {
    const RouteLeg& leg = route.legs[i];
    writer.StartObject();
    key_str("from", leg.from);
    key_str("to", leg.to);
    key_str("via", leg.via);
    // Only concurrency legs carry the full designator list; omit the key for
    // ordinary single-airway or DCT legs to keep the output lean.
    if (!leg.concurrent_airways.empty()) {
      key_str_array("concurrent_airways", leg.concurrent_airways);
    }
    writer.Key("distance_nm");
    writer.Double(leg.distance_nm);
    writer.Key("cumulative_nm");
    writer.Double(cumulative[i]);
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();
}

}  // namespace bf
