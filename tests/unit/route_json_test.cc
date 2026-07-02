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
  bf::RouteLeg concurrent;
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

TEST_CASE("ToString maps every connection kind", "[route_json]") {
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kProcedure)) == "procedure");
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kDirect)) == "direct");
  CHECK(std::string(bf::ToString(bf::ConnectionKind::kRadarVectors)) == "radar_vectors");
}

}  // namespace
