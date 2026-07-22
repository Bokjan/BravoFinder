#include "io/cache/bfdb_naming.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("naming: FormatBfdbName encodes the cycle", "[unit][naming]") {
  CHECK(bf::FormatBfdbName(2601) == "nav_2601.bfdb");
  CHECK(bf::FormatBfdbName(1) == "nav_1.bfdb");
}

TEST_CASE("naming: FormatBfdbName falls back to nav.bfdb without provenance", "[unit][naming]") {
  CHECK(bf::FormatBfdbName(0) == "nav.bfdb");
}

TEST_CASE("naming: ParseBfdbName round-trips a canonical name", "[unit][naming]") {
  auto parsed = bf::ParseBfdbName("nav_2601.bfdb");
  REQUIRE(parsed);
  CHECK(*parsed == 2601);
}

TEST_CASE("naming: ParseBfdbName round-trips the zero-cycle sentinel", "[unit][naming]") {
  // FormatBfdbName(0) -> "nav.bfdb" must parse back to 0, or a provenance-less
  // cache would be written under a name inventory discovery then drops.
  CHECK(bf::FormatBfdbName(0) == "nav.bfdb");
  auto parsed = bf::ParseBfdbName("nav.bfdb");
  REQUIRE(parsed);
  CHECK(*parsed == 0);
}

TEST_CASE("naming: ParseBfdbName inspects only the filename component", "[unit][naming]") {
  auto parsed = bf::ParseBfdbName("/some/dir/nav_2601.bfdb");
  REQUIRE(parsed);
  CHECK(*parsed == 2601);
}

TEST_CASE("naming: ParseBfdbName rejects non-matching names", "[unit][naming]") {
  CHECK_FALSE(bf::ParseBfdbName("nav_.bfdb"));               // empty cycle segment
  CHECK_FALSE(bf::ParseBfdbName("nav_ab.bfdb"));             // non-numeric
  CHECK_FALSE(bf::ParseBfdbName("nav_2601.txt"));            // wrong extension
  CHECK_FALSE(bf::ParseBfdbName("other_2601.bfdb"));         // wrong prefix
  CHECK_FALSE(bf::ParseBfdbName("nav_2601_20260112.bfdb"));  // legacy two-segment name
}

TEST_CASE("naming: ParseBfdbName rejects overflowing numbers", "[unit][naming]") {
  // 2^32 does not fit in uint32_t; must not silently wrap.
  CHECK_FALSE(bf::ParseBfdbName("nav_4294967296.bfdb"));
}
