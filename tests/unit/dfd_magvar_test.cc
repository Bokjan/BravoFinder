#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "io/loaders/dfd2/dfd2_loader.h"

using Catch::Approx;

// Lock the DFD v2 true->magnetic sign convention: variation is WEST-negative and
// magnetic = true - variation. A silent sign flip would rotate every 'T'
// (true-referenced) procedure leg's course by twice the local variation.
TEST_CASE("ToMagnetic: west variation raises magnetic above true", "[dfd_magvar]") {
  // 10 deg West -> magvar = -10 -> magnetic = true + 10.
  CHECK(bf::ToMagnetic(90.0, -10.0) == Approx(100.0));
  // 10 deg East -> magvar = +10 -> magnetic = true - 10.
  CHECK(bf::ToMagnetic(90.0, 10.0) == Approx(80.0));
  // Zero variation is the identity.
  CHECK(bf::ToMagnetic(123.0, 0.0) == Approx(123.0));
}

TEST_CASE("ToMagnetic: normalizes the result to a 0-to-360 range", "[dfd_magvar]") {
  CHECK(bf::ToMagnetic(5.0, 10.0) == Approx(355.0));   // 5 - 10 = -5 -> 355
  CHECK(bf::ToMagnetic(355.0, -10.0) == Approx(5.0));  // 355 + 10 = 365 -> 5
  CHECK(bf::ToMagnetic(0.0, 0.0) == Approx(0.0));
}
