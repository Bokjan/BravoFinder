#include "core/domain/coordinate.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// Reference great-circle distances are taken from well-known airport positions
// and cross-checked against online great-circle calculators (NM).
constexpr bf::Coordinate kJfk{40.6398, -73.7789};
constexpr bf::Coordinate kLax{33.9425, -118.4081};
constexpr bf::Coordinate kLhr{51.4706, -0.4619};

TEST_CASE("distance to self is zero", "[unit][coordinate]") {
  CHECK_THAT(kJfk.DistanceTo(kJfk), WithinAbs(0.0, 1e-9));
}

TEST_CASE("distance is symmetric", "[unit][coordinate]") {
  CHECK_THAT(kJfk.DistanceTo(kLax), WithinAbs(kLax.DistanceTo(kJfk), 1e-9));
}

TEST_CASE("JFK to LAX great-circle distance", "[unit][coordinate]") {
  // Actual JFK-LAX great-circle distance is ~2144 NM.
  CHECK_THAT(kJfk.DistanceTo(kLax), WithinRel(2144.0, 0.01));
}

TEST_CASE("JFK to LHR great-circle distance", "[unit][coordinate]") {
  // Actual JFK-LHR great-circle distance is ~2990 NM.
  CHECK_THAT(kJfk.DistanceTo(kLhr), WithinRel(2990.0, 0.01));
}

TEST_CASE("one degree of latitude is about 60 NM", "[unit][coordinate]") {
  // A degree of latitude spans ~60 NM (one nautical mile per arcminute).
  const bf::Coordinate a{0.0, 0.0};
  const bf::Coordinate b{1.0, 0.0};
  CHECK_THAT(a.DistanceTo(b), WithinRel(60.0, 0.005));
}

TEST_CASE("antipodal points are finite and near half-circumference", "[unit][coordinate]") {
  // Antipodal points push the haversine `a` term to ~1.0, where floating-point
  // error can push it just past 1.0 and make asin(sqrt(a)) NaN. The atan2 form
  // stays finite. Half-circumference is ~10800 NM.
  const bf::Coordinate a{40.0, -100.0};
  const bf::Coordinate b{-40.0, 80.0};  // antipode of (40, -100) is (-40, 80)
  const double d = a.DistanceTo(b);
  CHECK(std::isfinite(d));
  CHECK_THAT(d, WithinRel(10800.0, 0.01));
}

}  // namespace
