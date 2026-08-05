// SPDX-License-Identifier: LGPL-3.0-or-later
#include "core/domain/coordinate.h"

#include <algorithm>
#include <cmath>

namespace bf {

namespace {

double ToRadians(double degrees) { return degrees * kPi / kDegreesHalfCircle; }

}  // namespace

double Coordinate::DistanceTo(const Coordinate& other) const noexcept {
  const double lat1 = ToRadians(latitude);
  const double lat2 = ToRadians(other.latitude);
  const double d_lat = lat2 - lat1;
  const double d_lon = ToRadians(other.longitude - longitude);

  const double sin_lat = std::sin(d_lat / 2.0);
  const double sin_lon = std::sin(d_lon / 2.0);
  const double a = sin_lat * sin_lat + std::cos(lat1) * std::cos(lat2) * sin_lon * sin_lon;
  // atan2 form: numerically stable for antipodal/near-antipodal points where
  // floating-point error can push `a` just past 1.0 (sqrt(a) > 1 -> asin NaN).
  // Clamping the second argument keeps it finite; atan2 is well-defined for a
  // negative second argument.
  const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
  return kEarthRadiusNm * c;
}

double Coordinate::BearingTo(const Coordinate& other) const noexcept {
  const double lat1 = ToRadians(latitude);
  const double lat2 = ToRadians(other.latitude);
  const double d_lon = ToRadians(other.longitude - longitude);
  const double y = std::sin(d_lon) * std::cos(lat2);
  const double x =
      std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(d_lon);
  double deg = std::atan2(y, x) * kDegreesHalfCircle / kPi;
  if (deg < 0.0) {
    deg += kDegreesFullCircle;
  }
  return deg;
}

}  // namespace bf
