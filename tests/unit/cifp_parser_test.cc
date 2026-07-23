#include "io/loaders/xplane12/cifp_parser.h"

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

TEST_CASE("CIFP parser: DEEZZ5 RW31L leg sequence matches the file", "[unit][cifp]") {
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

TEST_CASE("CIFP parser: definite-fix legs resolve their fix and region", "[unit][cifp]") {
  const bf::CifpData data = bf::CifpParser::ParseLines(kDeezz5Rw31L);
  const bf::Procedure& p = data.procedures.front();

  // The CF leg flies to SKORR (region K6); it terminates at a fix.
  CHECK(p.legs[1].fix.IdentView() == "SKORR");
  CHECK(p.legs[1].fix.RegionView() == "K6");
  CHECK(p.legs[1].fix_is_definite());

  // The VI/VM legs have no fix and do not terminate at one.
  CHECK(p.legs[0].fix.IdentView().empty());
  CHECK_FALSE(p.legs[0].fix_is_definite());
  CHECK_FALSE(p.legs[4].fix_is_definite());
}

TEST_CASE("CIFP parser: course, distance, and altitude columns", "[unit][cifp]") {
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

TEST_CASE("CIFP parser: RNP, turn direction, and speed limit columns", "[unit][cifp]") {
  // Real cycle-2601 KJFK approach rows: an RF leg to JEVNI (right turn, RNP 0.30)
  // and a TF leg to PEEBO (speed limit 185 kt, RNP 0.30, no turn direction).
  const std::vector<std::string> lines = {
      "APPCH:027,R,R13L, ,JEVNI,K6,P,C,E   ,R,302,RF, , , , , ,002100,    ,    ,    ,0033,+,00313,"
      "     ,     , ,   ,-300,   ,CFBMG,K6,P,C, ,A,P,S;",
      "APPCH:023,R,R13RZ, ,PEEBO,K6,P,C,E   , ,302,TF, , , , , ,      ,    ,    ,0440,0034, ,     ,"
      "     ,     ,-,185,-300,   , , , , , ,B,J,S;",
  };
  const bf::CifpData data = bf::CifpParser::ParseLines(lines);
  REQUIRE(data.procedures.size() == 2);

  // JEVNI: right-turn RF leg, RNP 0.30 NM (302 -> 30 centinm), no speed limit.
  const bf::ProcedureLeg& jevni = data.procedures[0].legs.front();
  CHECK(jevni.fix.IdentView() == "JEVNI");
  CHECK(jevni.turn_dir == 'R');
  CHECK(jevni.rnp_centinm == 30);
  CHECK(jevni.speed_limit_kt == 0);

  // PEEBO: no turn direction, RNP 0.30 NM, 185 kt speed limit.
  const bf::ProcedureLeg& peebo = data.procedures[1].legs.front();
  CHECK(peebo.fix.IdentView() == "PEEBO");
  CHECK(peebo.turn_dir == '\0');
  CHECK(peebo.rnp_centinm == 30);
  CHECK(peebo.speed_limit_kt == 185);
}

TEST_CASE("CIFP parser: a change in transition starts a new procedure", "[unit][cifp]") {
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

TEST_CASE("CIFP parser: RWY records yield threshold coordinates", "[unit][cifp]") {
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

TEST_CASE("CIFP parser: all ARINC 424 path terminators are recognized", "[unit][cifp]") {
  // Round-trip every terminator token through parse + name. The full cycle-2601
  // corpus (14838 airports) uses all 23 of these; none must fall to kUnknown.
  const std::vector<std::string> tokens = {"TF", "IF", "DF", "CF", "AF", "RF", "CA", "FA",
                                           "VA", "HA", "CD", "FD", "VD", "CI", "VI", "CR",
                                           "VR", "FC", "FM", "VM", "PI", "HM", "HF"};
  for (const std::string& tok : tokens) {
    const bf::PathTerminator t = bf::ParsePathTerminator(tok);
    CHECK(t != bf::PathTerminator::kUnknown);
    CHECK(bf::PathTerminatorName(t) == tok);
  }
  CHECK(bf::ParsePathTerminator("ZZ") == bf::PathTerminator::kUnknown);
}
