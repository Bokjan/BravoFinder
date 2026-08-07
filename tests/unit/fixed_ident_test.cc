// SPDX-License-Identifier: MIT
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "core/domain/fixed_string.h"

// FixedIdent / FixedName8 back the sorted-vector indices that every
// binary-search lookup relies on, so their ordering and round-trip must be exact.

using bf::FixedIdent;
using bf::FixedName8;
using bf::Ident;

TEST_CASE("FixedIdent: round-trips ident and region", "[unit][fixed_ident]") {
  auto result = FixedIdent::FromParts("CANDR", "K6");
  REQUIRE(result);
  const FixedIdent f = result.value();
  CHECK(f.IdentView() == "CANDR");
  CHECK(f.Arinc424IcaoCodeView() == "K6");
  const Ident back = f.ToIdent();
  CHECK(back.ident == "CANDR");
  CHECK(back.arinc424_icao_code == "K6");
}

TEST_CASE("FixedIdent: FromIdent matches FromParts", "[unit][fixed_ident]") {
  auto a_result = FixedIdent::FromIdent(Ident("PSB", "K6"));
  auto b_result = FixedIdent::FromParts("PSB", "K6");
  REQUIRE(a_result);
  REQUIRE(b_result);
  const FixedIdent a = a_result.value();
  const FixedIdent b = b_result.value();
  CHECK(a == b);
}

TEST_CASE("FixedIdent: empty ident and region round-trip", "[unit][fixed_ident]") {
  auto result = FixedIdent::FromParts("", "");
  REQUIRE(result);
  const FixedIdent f = result.value();
  CHECK(f.IdentView().empty());
  CHECK(f.Arinc424IcaoCodeView().empty());
  CHECK(f.ToIdent().ident.empty());
  CHECK(f.ToIdent().arinc424_icao_code.empty());
}

TEST_CASE("FixedIdent: equality distinguishes ident and region", "[unit][fixed_ident]") {
  auto base_result = FixedIdent::FromParts("DGC", "K2");
  auto same_result = FixedIdent::FromParts("DGC", "K2");
  auto other_region_result = FixedIdent::FromParts("DGC", "LF");
  auto other_ident_result = FixedIdent::FromParts("DGD", "K2");
  auto shorter_ident_result = FixedIdent::FromParts("DG", "K2");
  REQUIRE(base_result);
  REQUIRE(same_result);
  REQUIRE(other_region_result);
  REQUIRE(other_ident_result);
  REQUIRE(shorter_ident_result);
  const FixedIdent base = base_result.value();
  CHECK(base == same_result.value());
  CHECK_FALSE(base == other_region_result.value());   // region differs
  CHECK_FALSE(base == other_ident_result.value());    // ident differs
  CHECK_FALSE(base == shorter_ident_result.value());  // length differs
}

TEST_CASE("FixedIdent: orders by ident then region", "[unit][fixed_ident]") {
  // Primary key is the ident; the region breaks ties.
  auto aaa_k2 = FixedIdent::FromParts("AAA", "K2");
  auto aab_k1 = FixedIdent::FromParts("AAB", "K1");
  auto aaa_k1 = FixedIdent::FromParts("AAA", "K1");
  REQUIRE(aaa_k2);
  REQUIRE(aab_k1);
  REQUIRE(aaa_k1);
  CHECK(aaa_k2.value() < aab_k1.value());
  CHECK(aaa_k1.value() < aaa_k2.value());
  CHECK_FALSE(aaa_k2.value() < aaa_k2.value());
  // A shorter ident that is a prefix sorts before the longer one.
  auto aa_k1 = FixedIdent::FromParts("AA", "K1");
  REQUIRE(aa_k1);
  CHECK(aa_k1.value() < aaa_k1.value());
}

TEST_CASE("FixedIdent: sort + binary search agree", "[unit][fixed_ident]") {
  auto psb = FixedIdent::FromParts("PSB", "K6");
  auto candr_k6 = FixedIdent::FromParts("CANDR", "K6");
  auto candr_k2 = FixedIdent::FromParts("CANDR", "K2");
  auto aaa = FixedIdent::FromParts("AAA", "K1");
  REQUIRE(psb);
  REQUIRE(candr_k6);
  REQUIRE(candr_k2);
  REQUIRE(aaa);
  std::vector<FixedIdent> v = {psb.value(), candr_k6.value(), candr_k2.value(), aaa.value()};
  std::sort(v.begin(), v.end());
  // std::is_sorted uses the same operator<; a broken strict-weak-ordering would
  // trip here or make the search below miss.
  CHECK(std::is_sorted(v.begin(), v.end()));
  auto key_result = FixedIdent::FromParts("CANDR", "K2");
  REQUIRE(key_result);
  const FixedIdent key = key_result.value();
  CHECK(std::binary_search(v.begin(), v.end(), key));
  auto absent_result = FixedIdent::FromParts("CANDR", "LF");
  REQUIRE(absent_result);
  const FixedIdent absent = absent_result.value();
  CHECK_FALSE(std::binary_search(v.begin(), v.end(), absent));
}

TEST_CASE("FixedName8: round-trips and orders", "[unit][fixed_ident]") {
  auto f_result = FixedName8::From("KIKR");
  auto knwl_result = FixedName8::From("KNWL");
  auto kik_result = FixedName8::From("KIK");
  REQUIRE(f_result);
  REQUIRE(knwl_result);
  REQUIRE(kik_result);
  const FixedName8 f = f_result.value();
  CHECK(f.View() == "KIKR");
  CHECK(f == f_result.value());
  CHECK_FALSE(f == knwl_result.value());

  // Ordering: memcmp then length, so a prefix sorts before the longer string.
  CHECK(kik_result.value() < f);
  CHECK(f < knwl_result.value());
  CHECK_FALSE(knwl_result.value() < knwl_result.value());
}

TEST_CASE("FixedName8: sort + binary search agree", "[unit][fixed_ident]") {
  auto zggg = FixedName8::From("ZGGG");
  auto kikr = FixedName8::From("KIKR");
  auto knwl = FixedName8::From("KNWL");
  auto zhhh = FixedName8::From("ZHHH");
  REQUIRE(zggg);
  REQUIRE(kikr);
  REQUIRE(knwl);
  REQUIRE(zhhh);
  std::vector<FixedName8> v = {zggg.value(), kikr.value(), knwl.value(), zhhh.value()};
  std::sort(v.begin(), v.end());
  CHECK(std::is_sorted(v.begin(), v.end()));
  CHECK(std::binary_search(v.begin(), v.end(), knwl.value()));
  auto kzzz = FixedName8::From("KZZZ");
  REQUIRE(kzzz);
  CHECK_FALSE(std::binary_search(v.begin(), v.end(), kzzz.value()));
}

TEST_CASE("FixedIdent and FixedName8 reject overflow", "[unit][fixed_ident]") {
  const auto ident = FixedIdent::FromParts("TOOLONG8", "K6");
  REQUIRE_FALSE(ident);
  CHECK(ident.error().code == bf::ErrorCode::kInvalidArgument);
  CHECK(ident.error().message.find("FixedIdent::kIdentCap") != std::string::npos);

  const auto region = FixedIdent::FromParts("FIX", "LONG");
  REQUIRE_FALSE(region);
  CHECK(region.error().code == bf::ErrorCode::kInvalidArgument);
  CHECK(region.error().message.find("FixedIdent::kArinc424IcaoCodeCap") != std::string::npos);

  const auto name = FixedName8::From("TOOLONG8");
  REQUIRE_FALSE(name);
  CHECK(name.error().code == bf::ErrorCode::kInvalidArgument);
  CHECK(name.error().message.find("FixedName::kCap") != std::string::npos);
}
