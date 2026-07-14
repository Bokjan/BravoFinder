#include "core/routing/route_metrics.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <vector>

#include "core/routing/route.h"

namespace {

bf::RouteLeg Leg(double dist) {
  bf::RouteLeg leg;
  leg.distance_nm = dist;
  return leg;
}

}  // namespace

TEST_CASE("CumulativeDistances runs a parallel prefix sum", "[route_metrics]") {
  const std::vector<bf::RouteLeg> legs = {Leg(3.4), Leg(100.0), Leg(50.6), Leg(12.0)};
  const std::vector<double> cum = bf::CumulativeDistances(legs);
  REQUIRE(cum.size() == legs.size());
  CHECK(cum[0] == Catch::Approx(3.4));
  CHECK(cum[1] == Catch::Approx(103.4));
  CHECK(cum[2] == Catch::Approx(154.0));
  CHECK(cum[3] == Catch::Approx(166.0));  // last element == route total
}

TEST_CASE("CumulativeDistances on no legs is empty", "[route_metrics]") {
  CHECK(bf::CumulativeDistances({}).empty());
}
