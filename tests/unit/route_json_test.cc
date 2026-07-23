#include "core/routing/route_json.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace {

// A minimal RapidJSON-shaped sink that records the call sequence as text. It
// proves WriteRouteJson is truly duck-typed (no RapidJSON header needed here)
// and lets us assert the emitted structure without a real JSON parser.
struct StubWriter {
  std::string out;
  void StartObject() { out += "{"; }
  void EndObject() { out += "}"; }
  void StartArray() { out += "["; }
  void EndArray() { out += "]"; }
  void Key(const char* k) { out += std::string("<") + k + ">"; }
  void String(const char* s, unsigned) { out += std::string("\"") + s + "\""; }
  void String(const char* s) { out += std::string("\"") + s + "\""; }
  void Double(double) { out += "#"; }
};

TEST_CASE("WriteRouteJson emits concurrent_airways only on concurrency legs",
          "[unit][route_json]") {
  bf::Route route;
  route.route_string = "A Y28 C";
  route.dep_connection = bf::ConnectionKind::kProcedure;
  route.arr_connection = bf::ConnectionKind::kDirect;

  bf::RouteLeg plain;
  plain.from = "A";
  plain.to = "B";
  plain.via = "Y28";
  bf::RouteLeg concurrent{};
  concurrent.from = "B";
  concurrent.to = "C";
  concurrent.via = "Y28";
  concurrent.concurrent_airways = {"V28", "Y28"};
  route.legs = {plain, concurrent};

  StubWriter w;
  bf::WriteRouteJson(w, route);

  // The single-airway leg omits the key; the concurrency leg includes it.
  const std::string& s = w.out;
  const size_t first_via = s.find("<via>");
  const size_t second_via = s.find("<via>", first_via + 1);
  REQUIRE(first_via != std::string::npos);
  REQUIRE(second_via != std::string::npos);
  // Exactly one concurrent_airways key overall.
  CHECK(s.find("<concurrent_airways>") != std::string::npos);
  CHECK(s.find("<concurrent_airways>", s.find("<concurrent_airways>") + 1) == std::string::npos);
  // The connection kinds serialize via bf::ToString.
  CHECK(s.find("\"procedure\"") != std::string::npos);
  CHECK(s.find("\"direct\"") != std::string::npos);
}

TEST_CASE("WriteRouteJson emits points[] with ident/lat/lon and per-leg cumulative_nm",
          "[unit][route_json]") {
  bf::Route route;
  route.route_string = "A Y28 C";

  bf::RoutePoint a;
  a.ident = "A";
  a.coord = {40.0, -73.0};
  bf::RoutePoint b;
  b.ident = "B";
  b.coord = {41.0, -74.0};
  bf::RoutePoint c;
  c.ident = "C";
  c.coord = {42.0, -75.0};
  route.points = {a, b, c};

  bf::RouteLeg leg1;
  leg1.from = "A";
  leg1.to = "B";
  leg1.via = "Y28";
  leg1.distance_nm = 10.0;
  bf::RouteLeg leg2;
  leg2.from = "B";
  leg2.to = "C";
  leg2.via = "Y28";
  leg2.distance_nm = 15.0;
  route.legs = {leg1, leg2};

  StubWriter w;
  bf::WriteRouteJson(w, route);
  const std::string& s = w.out;

  // The points array names ident/lat/lon for each point (N points).
  CHECK(s.find("<points>") != std::string::npos);
  CHECK(s.find("<ident>") != std::string::npos);
  CHECK(s.find("<lat>") != std::string::npos);
  CHECK(s.find("<lon>") != std::string::npos);
  // Each leg carries cumulative_nm alongside distance_nm (N-1 legs).
  const size_t first_cumul = s.find("<cumulative_nm>");
  const size_t second_cumul = s.find("<cumulative_nm>", first_cumul + 1);
  REQUIRE(first_cumul != std::string::npos);
  REQUIRE(second_cumul != std::string::npos);
  CHECK(s.find("<cumulative_nm>", second_cumul + 1) == std::string::npos);
}

TEST_CASE("ToString maps every connection kind", "[unit][route_json]") {
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kProcedure)) == "procedure");
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kDirect)) == "direct");
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kRadarVectors)) == "radar_vectors");
}

TEST_CASE("WriteRouteJson preserves coordinate precision with a 6-dp writer",
          "[unit][route_json]") {
  // The route object carries point lat/lon. A writer configured for 2 decimal
  // places (as the handlers used to be) would truncate a real coordinate such
  // as 40.639927 to 40.64 (~1.1 km). This drives the writer at 6 dp -- as the
  // CLI / MCP / HTTP handlers now do -- and asserts the precision survives.
  // (The StubWriter above discards Double values, so it cannot catch this; a
  // real RapidJSON writer is needed.)
  bf::Route route;
  route.route_string = "A Y28 C";
  bf::RoutePoint a;
  a.ident = "A";
  a.coord = {40.123456, -73.987654};
  route.points = {a};
  bf::RouteLeg leg;
  leg.from = "A";
  leg.to = "B";
  leg.via = "Y28";
  leg.distance_nm = 12.345678;
  route.legs = {leg};

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  bf::WriteRouteJson(writer, route);

  rapidjson::Document doc;
  doc.Parse(buffer.GetString());
  REQUIRE_FALSE(doc.HasParseError());
  const rapidjson::Value& point = doc["points"][0];
  // At 6 dp the coordinate round-trips to its full precision; a 2-dp writer
  // would have emitted 40.12 / -73.99, off by ~3e-3 -- well outside the margin.
  CHECK(point["lat"].GetDouble() == Catch::Approx(40.123456).margin(1e-6));
  CHECK(point["lon"].GetDouble() == Catch::Approx(-73.987654).margin(1e-6));
}

}  // namespace
