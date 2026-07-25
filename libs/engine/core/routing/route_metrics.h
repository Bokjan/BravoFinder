// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <vector>

#include "core/routing/route.h"

namespace bf {

// Small pure metrics derived from a computed Route, kept out of the plain-data
// route.h so the domain type stays a POD. Header-only: callers that already
// have RouteLeg link nothing extra.

// Running cumulative distance after each leg: element i is the sum of
// distance_nm for legs 0..i, so the last element equals the route total. The
// returned vector is parallel to `legs`.
inline std::vector<double> CumulativeDistances(const std::vector<RouteLeg>& legs) {
  std::vector<double> out;
  out.reserve(legs.size());
  double running = 0.0;
  for (const RouteLeg& leg : legs) {
    running += leg.distance_nm;
    out.push_back(running);
  }
  return out;
}

}  // namespace bf
