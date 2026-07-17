#include "core/domain/fixed_ident.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "core/domain/fixed_ident_no_region.h"

// FixedIdent / FixedIdentNoRegion back the sorted-vector indices that every
// binary-search lookup relies on, so their ordering and round-trip must be
// exact. (Overflow-clamp is deliberately not exercised here: it asserts in debug
// -- the build these tests run under -- and only clamps in release.)

using bf::FixedIdent;
using bf::FixedIdentNoRegion;
using bf::Ident;

TEST_CASE("FixedIdent: round-trips ident and region", "[fixed_ident]") {
  const FixedIdent f = FixedIdent::FromParts("CANDR", "K6");
  CHECK(f.IdentView() == "CANDR");
  CHECK(f.RegionView() == "K6");
  const Ident back = f.ToIdent();
  CHECK(back.ident == "CANDR");
  CHECK(back.region == "K6");
}

TEST_CASE("FixedIdent: FromIdent matches FromParts", "[fixed_ident]") {
  const FixedIdent a = FixedIdent::FromIdent(Ident("PSB", "K6"));
  const FixedIdent b = FixedIdent::FromParts("PSB", "K6");
  CHECK(a == b);
}

TEST_CASE("FixedIdent: empty ident and region round-trip", "[fixed_ident]") {
  const FixedIdent f = FixedIdent::FromParts("", "");
  CHECK(f.IdentView().empty());
  CHECK(f.RegionView().empty());
  CHECK(f.ToIdent().ident.empty());
  CHECK(f.ToIdent().region.empty());
}

TEST_CASE("FixedIdent: equality distinguishes ident and region", "[fixed_ident]") {
  const FixedIdent base = FixedIdent::FromParts("DGC", "K2");
  CHECK(base == FixedIdent::FromParts("DGC", "K2"));
  CHECK_FALSE(base == FixedIdent::FromParts("DGC", "LF"));  // region differs
  CHECK_FALSE(base == FixedIdent::FromParts("DGD", "K2"));  // ident differs
  CHECK_FALSE(base == FixedIdent::FromParts("DG", "K2"));   // length differs
}

TEST_CASE("FixedIdent: orders by ident then region", "[fixed_ident]") {
  // Primary key is the ident; the region breaks ties.
  CHECK(FixedIdent::FromParts("AAA", "K2") < FixedIdent::FromParts("AAB", "K1"));
  CHECK(FixedIdent::FromParts("AAA", "K1") < FixedIdent::FromParts("AAA", "K2"));
  CHECK_FALSE(FixedIdent::FromParts("AAA", "K2") < FixedIdent::FromParts("AAA", "K2"));
  // A shorter ident that is a prefix sorts before the longer one.
  CHECK(FixedIdent::FromParts("AA", "K1") < FixedIdent::FromParts("AAA", "K1"));
}

TEST_CASE("FixedIdent: sort + binary search agree", "[fixed_ident]") {
  std::vector<FixedIdent> v = {
      FixedIdent::FromParts("PSB", "K6"), FixedIdent::FromParts("CANDR", "K6"),
      FixedIdent::FromParts("CANDR", "K2"), FixedIdent::FromParts("AAA", "K1")};
  std::sort(v.begin(), v.end());
  // std::is_sorted uses the same operator<; a broken strict-weak-ordering would
  // trip here or make the search below miss.
  CHECK(std::is_sorted(v.begin(), v.end()));
  const FixedIdent key = FixedIdent::FromParts("CANDR", "K2");
  CHECK(std::binary_search(v.begin(), v.end(), key));
  const FixedIdent absent = FixedIdent::FromParts("CANDR", "LF");
  CHECK_FALSE(std::binary_search(v.begin(), v.end(), absent));
}

TEST_CASE("FixedIdentNoRegion: round-trips and orders", "[fixed_ident]") {
  const FixedIdentNoRegion f = FixedIdentNoRegion::From("KIKR");
  CHECK(f.View() == "KIKR");
  CHECK(f == FixedIdentNoRegion::From("KIKR"));
  CHECK_FALSE(f == FixedIdentNoRegion::From("KNWL"));

  // Ordering: memcmp then length, so a prefix sorts before the longer string.
  CHECK(FixedIdentNoRegion::From("KIK") < FixedIdentNoRegion::From("KIKR"));
  CHECK(FixedIdentNoRegion::From("KIKR") < FixedIdentNoRegion::From("KNWL"));
  CHECK_FALSE(FixedIdentNoRegion::From("KNWL") < FixedIdentNoRegion::From("KNWL"));
}

TEST_CASE("FixedIdentNoRegion: sort + binary search agree", "[fixed_ident]") {
  std::vector<FixedIdentNoRegion> v = {
      FixedIdentNoRegion::From("ZGGG"), FixedIdentNoRegion::From("KIKR"),
      FixedIdentNoRegion::From("KNWL"), FixedIdentNoRegion::From("ZHHH")};
  std::sort(v.begin(), v.end());
  CHECK(std::is_sorted(v.begin(), v.end()));
  CHECK(std::binary_search(v.begin(), v.end(), FixedIdentNoRegion::From("KNWL")));
  CHECK_FALSE(std::binary_search(v.begin(), v.end(), FixedIdentNoRegion::From("KZZZ")));
}
