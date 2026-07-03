#include "io/cache/bfdb_naming.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("naming: FormatBfdbName encodes cycle and build", "[unit][naming]") {
  CHECK(bf::FormatBfdbName(2601, 20260112) == "nav_2601_20260112.bfdb");
  CHECK(bf::FormatBfdbName(1, 2) == "nav_1_2.bfdb");
}

TEST_CASE("naming: FormatBfdbName falls back to nav.bfdb without provenance", "[unit][naming]") {
  CHECK(bf::FormatBfdbName(0, 0) == "nav.bfdb");
  // A partial (only one zero) is still a real cycle/build and keeps the pattern.
  CHECK(bf::FormatBfdbName(2601, 0) == "nav_2601_0.bfdb");
  CHECK(bf::FormatBfdbName(0, 20260112) == "nav_0_20260112.bfdb");
}

TEST_CASE("naming: ParseBfdbName round-trips a canonical name", "[unit][naming]") {
  auto parsed = bf::ParseBfdbName("nav_2601_20260112.bfdb");
  REQUIRE(parsed);
  CHECK(parsed->first == 2601);
  CHECK(parsed->second == 20260112);
}

TEST_CASE("naming: ParseBfdbName inspects only the filename component", "[unit][naming]") {
  auto parsed = bf::ParseBfdbName("/some/dir/nav_2601_20260112.bfdb");
  REQUIRE(parsed);
  CHECK(parsed->first == 2601);
  CHECK(parsed->second == 20260112);
}

TEST_CASE("naming: ParseBfdbName rejects the CIFP companion", "[unit][naming]") {
  // The companion must never be mistaken for a graph cache during a scan.
  CHECK_FALSE(bf::ParseBfdbName("nav_2601_20260112_cifp.bfdb"));
  CHECK_FALSE(bf::ParseBfdbName("nav_cifp.bfdb"));
}

TEST_CASE("naming: ParseBfdbName rejects non-matching names", "[unit][naming]") {
  CHECK_FALSE(bf::ParseBfdbName("nav.bfdb"));               // legacy fallback name, no cycle/build
  CHECK_FALSE(bf::ParseBfdbName("nav_2601.bfdb"));          // missing build segment
  CHECK_FALSE(bf::ParseBfdbName("nav_2601_.bfdb"));         // empty build segment
  CHECK_FALSE(bf::ParseBfdbName("nav__20260112.bfdb"));     // empty cycle segment
  CHECK_FALSE(bf::ParseBfdbName("nav_ab_cd.bfdb"));         // non-numeric
  CHECK_FALSE(bf::ParseBfdbName("nav_2601_20260112.txt"));  // wrong extension
  CHECK_FALSE(bf::ParseBfdbName("other_2601_20260112.bfdb"));  // wrong prefix
}

TEST_CASE("naming: ParseBfdbName rejects overflowing numbers", "[unit][naming]") {
  // 2^32 does not fit in uint32_t; must not silently wrap.
  CHECK_FALSE(bf::ParseBfdbName("nav_4294967296_1.bfdb"));
}
