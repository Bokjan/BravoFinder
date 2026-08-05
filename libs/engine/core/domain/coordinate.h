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
  double DistanceTo(const Coordinate& other) const noexcept;

  // Initial great-circle bearing to another coordinate, in degrees clockwise
  // from true north, normalized to [0, 360). Used by the turn-angle constraint
  // to compare the inbound and outbound headings at a path vertex. The initial
  // bearing (not the final bearing) is the right value for a turn at the
  // vertex: it is the heading the aircraft flies leaving one fix toward the
  // next, which is what a heading-continuity constraint compares against.
  double BearingTo(const Coordinate& other) const noexcept;
};

// Mean earth radius in nautical miles. 6371.0088 km is the IUGG mean radius;
// one nautical mile is exactly 1.852 km. Shared by the haversine distance and
// by chord-based A* heuristics so a chord length (a lower bound on the arc) and
// the true arc use the same radius and stay consistent.
inline constexpr double kEarthRadiusNm = 6371.0088 / 1.852;

// Angle conversion constants shared by haversine, bearing, MORA sampling, and
// the turn-angle penalty (degree/radian and full/half-circle wraps).
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegreesFullCircle = 360.0;
inline constexpr double kDegreesHalfCircle = 180.0;

}  // namespace bf
