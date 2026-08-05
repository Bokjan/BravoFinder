// SPDX-License-Identifier: MIT
#include <catch2/catch_test_macros.hpp>

#include "core/domain/procedure.h"

TEST_CASE("IsMaptDesc: WD char4 M is MAPT; EYM is not", "[unit][procedure]") {
  CHECK(bf::IsMaptDesc("EY M"));
  CHECK(bf::IsMaptDesc("E  M"));
  CHECK(bf::IsMaptDesc("GY M"));
  CHECK_FALSE(bf::IsMaptDesc("EYM"));   // missed-approach start (len 3)
  CHECK_FALSE(bf::IsMaptDesc("EE H"));  // holding
  CHECK_FALSE(bf::IsMaptDesc("E  A"));  // IAF
  CHECK_FALSE(bf::IsMaptDesc(""));
  CHECK_FALSE(bf::IsMaptDesc("E"));
}

TEST_CASE("NormalizeRunwayIdent: bare digits gain RW prefix", "[unit][procedure]") {
  CHECK(bf::NormalizeRunwayIdent("18") == "RW18");
  CHECK(bf::NormalizeRunwayIdent("06L") == "RW06L");
  CHECK(bf::NormalizeRunwayIdent("RW31L") == "RW31L");
  CHECK(bf::NormalizeRunwayIdent("").empty());
}

TEST_CASE("ApproachRunwayFromName: type prefix plus runway digits", "[unit][procedure]") {
  CHECK(bf::ApproachRunwayFromName("R18") == "RW18");
  CHECK(bf::ApproachRunwayFromName("I14") == "RW14");
  CHECK(bf::ApproachRunwayFromName("R14-B") == "RW14");
  CHECK(bf::ApproachRunwayFromName("R10LY") == "RW10L");
  CHECK(bf::ApproachRunwayFromName("R10LZ") == "RW10L");
  CHECK(bf::ApproachRunwayFromName("I13L") == "RW13L");
  // Circling / no runway digits → empty (exempt from runway filter).
  CHECK(bf::ApproachRunwayFromName("CNDB").empty());
  CHECK(bf::ApproachRunwayFromName("VORA").empty());
  CHECK(bf::ApproachRunwayFromName("").empty());
}

TEST_CASE("ParseRouteTypeToken: numeric and single-alpha", "[unit][procedure]") {
  CHECK(bf::ParseRouteTypeToken("4") == 4);
  CHECK(bf::ParseRouteTypeToken("A") == static_cast<int>('A'));
  CHECK(bf::ParseRouteTypeToken("R") == static_cast<int>('R'));
  CHECK(bf::ParseRouteTypeToken("") == 0);
  CHECK(bf::ParseRouteTypeToken("AB") == 0);
}
