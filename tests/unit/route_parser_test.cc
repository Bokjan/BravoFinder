// SPDX-License-Identifier: MIT
#include "core/routing/route_parser.h"

#include <catch2/catch_test_macros.hpp>

TEST_CASE("tokenize: splits on whitespace and upper-cases", "[unit][route_parser]") {
  const auto t = bf::TokenizeRoute("kjfk deezz5 candr J60 psb klax");
  REQUIRE(t.size() == 6);
  CHECK(t[0] == "KJFK");
  CHECK(t[1] == "DEEZZ5");
  CHECK(t[2] == "CANDR");
  CHECK(t[3] == "J60");
  CHECK(t[4] == "PSB");
  CHECK(t[5] == "KLAX");
}

TEST_CASE("tokenize: collapses runs of whitespace and tabs", "[unit][route_parser]") {
  const auto t = bf::TokenizeRoute("  KJFK\t\tJ60   PSB \n KLAX  ");
  REQUIRE(t.size() == 4);
  CHECK(t[0] == "KJFK");
  CHECK(t[1] == "J60");
  CHECK(t[2] == "PSB");
  CHECK(t[3] == "KLAX");
}

TEST_CASE("tokenize: empty and whitespace-only strings yield no tokens", "[unit][route_parser]") {
  CHECK(bf::TokenizeRoute("").empty());
  CHECK(bf::TokenizeRoute("   \t\n ").empty());
}

TEST_CASE("tokenize: preserves IDENT/REGION and hyphenated designators", "[unit][route_parser]") {
  const auto t = bf::TokenizeRoute("PSB/K6 A1-G581");
  REQUIRE(t.size() == 2);
  CHECK(t[0] == "PSB/K6");
  CHECK(t[1] == "A1-G581");
}
