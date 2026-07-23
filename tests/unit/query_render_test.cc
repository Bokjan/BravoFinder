// query_render_test.cc — unit tests for the query-layer renderers (render.cc).
//
// The text renderers were ported verbatim from the former CLI printers, and the
// CLI text output had no test coverage before; these guard the byte-for-byte
// shape (labels, separators, the NDB-kHz / VOR-MHz split, the hold turn/leg
// branches, the multi-route header) against silent drift. Pure-function tests:
// no NavDatabase, no real data, so they run in the unit subset.

#include <rapidjson/document.h>

#include <catch2/catch_test_macros.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/query/query_types.h"
#include "core/routing/route.h"
#include "render.h"

namespace {

bf::service::OutputFormat Json() { return bf::service::OutputFormat::kJson; }
bf::service::OutputFormat Text() { return bf::service::OutputFormat::kText; }

}  // namespace

TEST_CASE("RenderError: json object vs text line", "[unit][render]") {
  CHECK(bf::service::RenderError(Json(), "nope") == R"({"error":"nope"})");
  CHECK(bf::service::RenderError(Text(), "nope") == "error: nope\n");
}

TEST_CASE("RenderWaypoints: text hit/miss and json grouped array", "[unit][render]") {
  bf::WaypointInfo w;
  w.ident = "NINOX";
  w.region = "ZB";
  w.coord = bf::Coordinate{40.64, -73.78};
  w.kind = bf::WaypointKind::kFix;
  w.on_network = true;
  const std::string text = bf::service::RenderWaypoints(Text(), {"NINOX", "NOPE"}, {{w}, {}});
  CHECK(text == "NINOX (ZB) fix  40.64, -73.78  [on-network]\nNOPE: not found\n");

  const std::string json = bf::service::RenderWaypoints(Json(), {"NINOX", "NOPE"}, {{w}, {}});
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  REQUIRE(doc.IsArray());
  REQUIRE(doc.Size() == 2);
  REQUIRE(doc[0].IsArray());         // grouped: each id is itself an array
  CHECK(doc[1].GetArray().Empty());  // miss = empty group, not null
}

TEST_CASE("RenderAirports: text carries elevation and procedure flag", "[unit][render]") {
  bf::AirportInfo a;
  a.icao = "KJFK";
  a.region = "K6";
  a.coord = bf::Coordinate{40.64, -73.78};
  a.elevation_ft = 13;
  a.has_procedures = true;
  const std::string text = bf::service::RenderAirports(Text(), {"KJFK"}, {a});
  CHECK(text == "KJFK (K6)  40.64, -73.78  elev 13 ft  [has procedures]\n");
}

TEST_CASE("RenderNavaidDetails: NDB kHz vs VOR MHz split", "[unit][render]") {
  bf::NavaidDetailInfo ndb;
  ndb.ident = "LV";
  ndb.region = "ZB";
  ndb.kind = bf::WaypointKind::kNdb;
  ndb.elev_ft = 0;
  ndb.freq_raw = 350;  // NDB: kHz
  ndb.range_nm = 50.0;

  bf::NavaidDetailInfo vor;
  vor.ident = "EWC";
  vor.region = "ZB";
  vor.kind = bf::WaypointKind::kVor;
  vor.elev_ft = 0;
  vor.freq_raw = 11500;  // VOR: MHz * 100 -> 115.0 MHz
  vor.range_nm = 200.0;

  const std::string text = bf::service::RenderNavaidDetails(Text(), {"LV", "EWC"}, {{ndb}, {vor}});
  CHECK(text.find("freq 350 kHz") != std::string::npos);
  CHECK(text.find("freq 115 MHz") != std::string::npos);
}

TEST_CASE("RenderHolds: leg distance, turn, altitude range, speed limit", "[unit][render]") {
  bf::HoldInfo h;
  h.fix_ident = "AE701";
  h.fix_region = "ZB";
  h.airport_icao = "ENRT";
  h.inbound_course = 180.0;
  h.leg_dist_nm = 1.5;
  h.turn_dir = 'R';
  h.min_alt_ft = 3000;
  h.max_alt_ft = 6000;
  h.speed_limit_kt = 250;
  const std::string text = bf::service::RenderHolds(Text(), {"AE701"}, {{h}});
  CHECK(text == "AE701 (ZB)  ENRT  inbound 180°  out 1.5 NM  R-turn  alt 3000-6000 ft  250 kt\n");
}

TEST_CASE("RenderHolds: timing leg falls back to minutes", "[unit][render]") {
  bf::HoldInfo h;
  h.fix_ident = "H";
  h.fix_region = "ZB";
  h.airport_icao = "ENRT";
  h.inbound_course = 90.0;
  h.leg_time_min = 1.0;  // leg_dist_nm == 0 -> use minutes
  h.turn_dir = 'L';
  h.min_alt_ft = 2000;
  h.max_alt_ft = 0;
  const std::string text = bf::service::RenderHolds(Text(), {"H"}, {{h}});
  CHECK(text.find("out 1 min") != std::string::npos);
  CHECK(text.find("L-turn") != std::string::npos);
}

TEST_CASE("RenderAirways: directed segment line", "[unit][render]") {
  bf::AirwayInfo a;
  a.name = "Y28";
  bf::AirwayLeg s;
  s.from = "ABC";
  s.to = "DEF";
  s.distance_nm = 12.5;
  s.high = false;
  s.base_fl = 120;
  s.top_fl = 180;
  a.segments.push_back(s);
  const std::string text = bf::service::RenderAirways(Text(), {"Y28"}, {a});
  CHECK(text == "Y28: 1 segments\n  ABC -> DEF  12.5 NM  low  FL120-180\n");
}

TEST_CASE("RenderProcedures: summary list text", "[unit][render]") {
  bf::AirportProcedures ap;
  ap.icao = "KJFK";
  bf::ProcedureSummary p;
  p.type = bf::ProcedureType::kSid;
  p.name = "DEEZZ5";
  p.transition = "RW31L";
  ap.procedures.push_back(p);
  const std::string text = bf::service::RenderProcedures(Text(), {"KJFK"}, {ap});
  CHECK(text == "KJFK: 1 procedures\n  sid DEEZZ5.RW31L\n");
}

TEST_CASE("RenderProcedureDetail: per-leg block", "[unit][render]") {
  bf::AirportProcedureDetail d;
  d.icao = "KJFK";
  d.procedure = "DEEZZ5";
  bf::ProcedureDetail t;
  t.type = bf::ProcedureType::kSid;
  t.name = "DEEZZ5";
  t.transition = "RW31L";
  bf::ProcedureLegInfo leg;
  leg.fix = "CANDR";
  leg.path_term = "TF";
  leg.course_deg = 90.0;
  leg.distance_nm = 5.0;
  t.legs.push_back(leg);
  d.transitions.push_back(t);
  const std::string text = bf::service::RenderProcedureDetail(Text(), d);
  CHECK(text.find("KJFK/DEEZZ5: 1 transitions") != std::string::npos);
  CHECK(text.find("SID DEEZZ5.RW31L  rwy RW31L") == std::string::npos);  // transition, not runway
  CHECK(text.find("TF CANDR  crs 90.0  5.0 NM") != std::string::npos);
}

TEST_CASE("RenderRoutes: single route text has no header and ends with elapsed", "[unit][render]") {
  bf::Route r;
  r.route_string = "KJFK SID CANDR J60 PSB STAR KLAX";
  r.total_distance_nm = 2160.0;
  r.dep_distance_nm = 10.0;
  r.enroute_distance_nm = 2140.0;
  r.arr_distance_nm = 10.0;
  const std::string text = bf::service::RenderRoutes(Text(), {r}, 42);
  CHECK(text.find("=== Route") == std::string::npos);  // single route: no header
  CHECK(text.find("Total distance: 2160.0 NM") != std::string::npos);
  CHECK(text.find("Query elapsed: 42 ms") != std::string::npos);
}

TEST_CASE("RenderRoutes: multiple routes get a numbered header per route", "[unit][render]") {
  bf::Route r;
  r.route_string = "A B C";
  const std::string text = bf::service::RenderRoutes(Text(), {r, r}, 7);
  CHECK(text.find("=== Route 1 of 2 ===") != std::string::npos);
  CHECK(text.find("=== Route 2 of 2 ===") != std::string::npos);
}

TEST_CASE("RenderRoutes: json is a bare array (the transport shape)", "[unit][render]") {
  bf::Route r;
  r.route_string = "A B C";
  const std::string json = bf::service::RenderRoutes(Json(), {r}, 7);
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  REQUIRE(doc.IsArray());
  REQUIRE(doc.Size() == 1);
  // elapsed_ms is out-of-band (in HandlerResult), not in the JSON body.
  CHECK(json.find("elapsed_ms") == std::string::npos);
}
