// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

namespace bf {

// A geographic position in decimal degrees (WGS-84).
//
// Latitude is positive north, longitude is positive east. Coordinate is an
// immutable value type; all members are public because it carries no invariant
// beyond holding two numbers.
struct Coordinate {
  double latitude = 0.0;   // degrees, [-90, 90]
  double longitude = 0.0;  // degrees, [-180, 180]

  // Great-circle distance to another coordinate, in nautical miles (NM),
  // computed with the haversine formula on a spherical earth model.
  double DistanceTo(const Coordinate& other) const;
};

// Mean earth radius in nautical miles. 6371.0088 km is the IUGG mean radius;
// one nautical mile is exactly 1.852 km. Shared by the haversine distance and
// by chord-based A* heuristics so a chord length (a lower bound on the arc) and
// the true arc use the same radius and stay consistent.
inline constexpr double kEarthRadiusNm = 6371.0088 / 1.852;

}  // namespace bf
