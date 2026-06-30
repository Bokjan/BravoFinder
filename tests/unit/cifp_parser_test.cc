#include "io/loaders/xplane/cifp/cifp_parser.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <vector>

#include "core/domain/procedure.h"

using Catch::Matchers::WithinAbs;

namespace {

// Real CIFP lines from cycle 2601 (KJFK). Used as a small real-format sample,
// not a mock: these are verbatim rows the parser must handle.
const std::vector<std::string> kDeezz5Rw31L = {
    "SID:010,4,DEEZZ5,RW31L, , , , ,    , ,   ,VI, , , , , ,      ,    ,    ,3140,    , ,     ,    "
    " "
    ",18000, ,   ,    ,   , , , , , , , , ;",
    "SID:020,4,DEEZZ5,RW31L,SKORR,K6,E,A,E   , ,   ,CF, ,CCC,K6,D, ,      ,2624,0536,2370,0040, ,  "
    "   ,     ,     , ,   ,    ,   , , , , , , , , ;",
    "SID:030,4,DEEZZ5,RW31L,CESID,K6,E,A,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    "
    ",+,02500,"
    "     ,     , ,   ,    ,   , , , , , , , , ;",
    "SID:040,4,DEEZZ5,RW31L,YNKEE,K6,E,A,EY  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     "
    ", "
    "    ,     , ,   ,    ,   , , , , , , , , ;",
    "SID:050,4,DEEZZ5,RW31L, , , , ,    , ,   ,VM, , , , , ,      ,    ,    ,1720,    , ,     ,    "
    " ,"
    "     , ,   ,    ,   , , , , , , , , ;",
    "SID:060,4,DEEZZ5,RW31L,DEEZZ,K6,E,A,EE H, ,   ,DF, , , , , ,      ,    ,    ,    ,    , ,     "
    ", "
    "    ,     , ,   ,    ,   , , , , , , , , ;",
};

}  // namespace

TEST_CASE("CIFP parser: DEEZZ5 RW31L leg sequence matches the file", "[cifp]") {
  const bf::CifpData data = bf::CifpParser::ParseLines(kDeezz5Rw31L);
  REQUIRE(data.procedures.size() == 1);

  const bf::Procedure& p = data.procedures.front();
  CHECK(p.type == bf::ProcedureType::kSid);
  CHECK(p.name == "DEEZZ5");
  CHECK(p.transition_ident == "RW31L");
  CHECK(p.runway == "RW31L");
  CHECK(p.route_type == 4);
  REQUIRE(p.legs.size() == 6);

  // Path-terminator sequence as published: VI CF TF TF VM DF.
  CHECK(p.legs[0].path_term == bf::PathTerminator::kVI);
  CHECK(p.legs[1].path_term == bf::PathTerminator::kCF);
  CHECK(p.legs[2].path_term == bf::PathTerminator::kTF);
  CHECK(p.legs[3].path_term == bf::PathTerminator::kTF);
  CHECK(p.legs[4].path_term == bf::PathTerminator::kVM);
  CHECK(p.legs[5].path_term == bf::PathTerminator::kDF);
}

TEST_CASE("CIFP parser: definite-fix legs resolve their fix and region", "[cifp]") {
  const bf::CifpData data = bf::CifpParser::ParseLines(kDeezz5Rw31L);
  const bf::Procedure& p = data.procedures.front();

  // The CF leg flies to SKORR (region K6); it terminates at a fix.
  CHECK(p.legs[1].fix.ident == "SKORR");
  CHECK(p.legs[1].fix.region == "K6");
  CHECK(p.legs[1].fix_is_definite());

  // The VI/VM legs have no fix and do not terminate at one.
  CHECK(p.legs[0].fix.ident.empty());
  CHECK_FALSE(p.legs[0].fix_is_definite());
  CHECK_FALSE(p.legs[4].fix_is_definite());
}

TEST_CASE("CIFP parser: course, distance, and altitude columns", "[cifp]") {
  const bf::CifpData data = bf::CifpParser::ParseLines(kDeezz5Rw31L);
  const bf::Procedure& p = data.procedures.front();

  // CF leg: course 237.0 deg, distance 4.0 NM (columns stored as tenths).
  CHECK_THAT(p.legs[1].course_deg, WithinAbs(237.0, 1e-6));
  CHECK_THAT(p.legs[1].distance_nm, WithinAbs(4.0, 1e-6));

  // TF leg to CESID carries "+02500": cross at or above 2500 ft.
  CHECK(p.legs[2].alt.kind == bf::AltConstraintKind::kAtOrAbove);
  CHECK(p.legs[2].alt.alt1_ft == 2500);

  // The first VI leg has no altitude restriction of its own.
  CHECK(p.legs[0].alt.kind == bf::AltConstraintKind::kNone);
}

TEST_CASE("CIFP parser: a change in transition starts a new procedure", "[cifp]") {
  std::vector<std::string> lines = {
      "SID:010,5,DEEZZ5, ,DEEZZ,K6,E,A,E  H, ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,     , "
      " "
      "   ,18000, ,   ,    ,   , , , , , , , , ;",
      "SID:020,5,DEEZZ5, ,HEERO,K6,E,A,EE  , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,     , "
      "  "
      "  ,     , ,   ,    ,   , , , , , , , , ;",
      "SID:010,6,DEEZZ5,CANDR,HEERO,K6,E,A,E   , ,   ,IF, , , , , ,      ,    ,    ,    ,    , ,   "
      "  "
      ",     ,18000, ,   ,    ,   , , , , , , , , ;",
      "SID:020,6,DEEZZ5,CANDR,KURNL,K6,E,A,E   , ,   ,TF, , , , , ,      ,    ,    ,    ,    , ,   "
      "  "
      ",     ,     , ,   ,    ,   , , , , , , , , ;",
  };
  const bf::CifpData data = bf::CifpParser::ParseLines(lines);
  REQUIRE(data.procedures.size() == 2);
  // Common segment (route type 5, blank transition) then CANDR enroute
  // transition (route type 6).
  CHECK(data.procedures[0].transition_ident.empty());
  CHECK(data.procedures[0].route_type == 5);
  CHECK(data.procedures[1].transition_ident == "CANDR");
  CHECK(data.procedures[1].route_type == 6);
}

TEST_CASE("CIFP parser: RWY records yield threshold coordinates", "[cifp]") {
  const std::vector<std::string> lines = {
      "RWY:RW04L,     ,      ,00012, ,IHIQ,1,   ;N40372318,W073470505,0460;",
  };
  const bf::CifpData data = bf::CifpParser::ParseLines(lines);
  REQUIRE(data.runways.size() == 1);
  const bf::Runway& r = data.runways.front();
  CHECK(r.ident == "RW04L");
  // N40 37 23.18 -> 40.6231 deg; W073 47 05.05 -> -73.7847 deg.
  CHECK_THAT(r.threshold.latitude, WithinAbs(40.6231, 1e-3));
  CHECK_THAT(r.threshold.longitude, WithinAbs(-73.7847, 1e-3));
}

TEST_CASE("CIFP parser: all fourteen path terminators are recognized", "[cifp]") {
  // Round-trip every terminator token through parse + name.
  const std::vector<std::string> tokens = {"TF", "IF", "DF", "CF", "CA", "FM", "VA",
                                           "VM", "VI", "VR", "RF", "VD", "HM", "HF"};
  for (const std::string& tok : tokens) {
    const bf::PathTerminator t = bf::ParsePathTerminator(tok);
    CHECK(t != bf::PathTerminator::kUnknown);
    CHECK(bf::PathTerminatorName(t) == tok);
  }
  CHECK(bf::ParsePathTerminator("ZZ") == bf::PathTerminator::kUnknown);
}
