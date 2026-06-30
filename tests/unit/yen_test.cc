#include <catch2/catch_test_macros.hpp>

#include "core/graph/yen_kshortest.h"
#include "io/graph_builder.h"

namespace {

// Build a small network offering several distinct A->D paths so Yen has real
// alternatives. Coordinates are spread along the equator; airways are two-way.
//
//   AAA -- BBB -- DDD     (via Q1 / Q2)
//   AAA -- CCC -- DDD     (via Q3 / Q4)
//
bf::NavData MakeDiamondData() {
  bf::NavData d;
  auto wp = [](const char* id, double lon) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{0.0, lon}, bf::WaypointKind::kFix};
  };
  // BBB slightly closer to the direct line than CCC, so the BBB path is shorter.
  d.waypoints = {wp("AAA", 0.0), wp("BBB", 1.0), wp("CCC", 1.0), wp("DDD", 2.0)};
  d.waypoints[1].coord = bf::Coordinate{0.1, 1.0};   // BBB
  d.waypoints[2].coord = bf::Coordinate{-0.5, 1.0};  // CCC (longer detour)

  auto seg = [](const char* name) {
    bf::AirwaySegment s;
    s.name = name;
    s.direction = bf::AirwayDirection::kBoth;
    return s;
  };
  d.airways = {
      {bf::Ident("AAA", "ZZ"), bf::Ident("BBB", "ZZ"), seg("Q1")},
      {bf::Ident("BBB", "ZZ"), bf::Ident("DDD", "ZZ"), seg("Q2")},
      {bf::Ident("AAA", "ZZ"), bf::Ident("CCC", "ZZ"), seg("Q3")},
      {bf::Ident("CCC", "ZZ"), bf::Ident("DDD", "ZZ"), seg("Q4")},
  };
  return d;
}

TEST_CASE("Yen returns K distinct paths ordered by cost", "[yen]") {
  bf::GraphBuilder builder(MakeDiamondData());
  const int a = builder.VertexByIdent("AAA");
  const int dd = builder.VertexByIdent("DDD");
  REQUIRE(a >= 0);
  REQUIRE(dd >= 0);

  std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPaths(builder.graph(), a, dd, 3, bf::SearchOptions{});

  // Two genuinely distinct A->D paths exist (via BBB and via CCC).
  REQUIRE(paths.size() == 2);
  CHECK(paths[0].found);
  // Ordered shortest-first.
  CHECK(paths[0].cost <= paths[1].cost);
  // The two paths differ.
  CHECK(paths[0].vertices != paths[1].vertices);
  // The shorter one goes through BBB.
  const int bbb = builder.VertexByIdent("BBB");
  CHECK(paths[0].vertices[1] == bbb);
}

TEST_CASE("Yen with k=1 returns just the best path", "[yen]") {
  bf::GraphBuilder builder(MakeDiamondData());
  const int a = builder.VertexByIdent("AAA");
  const int dd = builder.VertexByIdent("DDD");
  std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPaths(builder.graph(), a, dd, 1, bf::SearchOptions{});
  REQUIRE(paths.size() == 1);
}

TEST_CASE("Yen on unreachable goal returns empty", "[yen]") {
  bf::NavData d;
  d.waypoints = {bf::Waypoint{bf::Ident("AAA", "ZZ"), bf::Coordinate{0, 0}, {}},
                 bf::Waypoint{bf::Ident("BBB", "ZZ"), bf::Coordinate{0, 5}, {}}};
  bf::GraphBuilder builder(d);
  const int a = builder.VertexByIdent("AAA");
  const int b = builder.VertexByIdent("BBB");
  std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPaths(builder.graph(), a, b, 3, bf::SearchOptions{});
  CHECK(paths.empty());
}

}  // namespace
