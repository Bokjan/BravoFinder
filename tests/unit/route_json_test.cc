#include "core/routing/route_json.h"

#include <catch2/catch_test_macros.hpp>
#include <string>

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

TEST_CASE("WriteRouteJson emits concurrent_airways only on concurrency legs", "[route_json]") {
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
          "[route_json]") {
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

TEST_CASE("ToString maps every connection kind", "[route_json]") {
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kProcedure)) == "procedure");
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kDirect)) == "direct");
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kRadarVectors)) == "radar_vectors");
}

}  // namespace
