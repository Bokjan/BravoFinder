#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "core/graph/astar.h"
#include "io/graph_builder.h"

using Catch::Matchers::WithinRel;

TEST_CASE("GraphEdge stays compact (16 bytes)", "[graph]") {
  // The edge array is the largest structure in the graph and A* walks it on the
  // hot path; keeping the edge at 16 bytes doubles how many fit in a cache line.
  STATIC_REQUIRE(sizeof(bf::GraphEdge) == 16);
}

namespace {

// Build a tiny dataset by hand: four waypoints in a line A-B-C-D connected by a
// single two-way airway, plus a one-way airway used to test directionality.
//
//   A --- B --- C --- D     (airway "Q1", both directions)
//
bf::NavData MakeLineData() {
  bf::NavData d;
  auto wp = [](const char* id, double lat, double lon) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{lat, lon}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("AAA", 0.0, 0.0), wp("BBB", 0.0, 1.0), wp("CCC", 0.0, 2.0),
                 wp("DDD", 0.0, 3.0)};
  auto seg = [](const char* name, bf::AirwayDirection dir) {
    bf::AirwaySegment s;
    s.name = name;
    s.direction = dir;
    return s;
  };
  d.airways = {
      {bf::Ident("AAA", "ZZ"), bf::Ident("BBB", "ZZ"), seg("Q1", bf::AirwayDirection::kBoth)},
      {bf::Ident("BBB", "ZZ"), bf::Ident("CCC", "ZZ"), seg("Q1", bf::AirwayDirection::kBoth)},
      {bf::Ident("CCC", "ZZ"), bf::Ident("DDD", "ZZ"), seg("Q1", bf::AirwayDirection::kBoth)},
  };
  return d;
}

TEST_CASE("A* finds path along a linear airway", "[graph]") {
  bf::GraphBuilder builder(MakeLineData());
  const int a = builder.VertexByIdent("AAA");
  const int d = builder.VertexByIdent("DDD");
  REQUIRE(a >= 0);
  REQUIRE(d >= 0);

  bf::ShortestPath path = bf::FindShortestPath(builder.graph(), a, d);
  REQUIRE(path.found);
  // A-B-C-D = 4 vertices.
  CHECK(path.vertices.size() == 4);
  // Three degrees of longitude at the equator is ~180 NM.
  CHECK_THAT(path.distance_nm, WithinRel(180.0, 0.01));
}

TEST_CASE("one-way airway is not traversable backward", "[graph]") {
  bf::NavData d;
  auto wp = [](const char* id, double lon) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{0.0, lon}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("AAA", 0.0), wp("BBB", 1.0)};
  bf::AirwaySegment fwd;
  fwd.name = "F1";
  fwd.direction = bf::AirwayDirection::kForward;  // AAA -> BBB only
  d.airways = {{bf::Ident("AAA", "ZZ"), bf::Ident("BBB", "ZZ"), fwd}};

  bf::GraphBuilder builder(d);
  const int a = builder.VertexByIdent("AAA");
  const int b = builder.VertexByIdent("BBB");

  // Forward is allowed...
  CHECK(bf::FindShortestPath(builder.graph(), a, b).found);
  // ...backward is not.
  CHECK_FALSE(bf::FindShortestPath(builder.graph(), b, a).found);
}

TEST_CASE("unreachable vertices report no path", "[graph]") {
  bf::NavData d;
  d.waypoints = {bf::Waypoint{bf::Ident("AAA", "ZZ"), bf::Coordinate{0, 0}, {}},
                 bf::Waypoint{bf::Ident("BBB", "ZZ"), bf::Coordinate{10, 10}, {}}};
  // No airways: the two points are disconnected.
  bf::GraphBuilder builder(d);
  const int a = builder.VertexByIdent("AAA");
  const int b = builder.VertexByIdent("BBB");
  CHECK_FALSE(bf::FindShortestPath(builder.graph(), a, b).found);
}

TEST_CASE("multi-source/goal A* picks the cheapest seeded combination", "[graph]") {
  // A-B-C-D in a line (~60 NM per degree-step at the equator). Sources seed at
  // A and B; goals seed at C and D. The search must weigh seed costs against
  // enroute distance to choose the best end-to-end combination.
  bf::GraphBuilder builder(MakeLineData());
  const int a = builder.VertexByIdent("AAA");
  const int b = builder.VertexByIdent("BBB");
  const int c = builder.VertexByIdent("CCC");
  const int d = builder.VertexByIdent("DDD");
  const bf::NavGraph& g = builder.graph();

  SECTION("cheap seeds at the near pair win") {
    // Source B (seed 0) -> goal C (seed 0): just the B-C enroute leg (~60 NM).
    // Source A (seed 0) would add the A-B leg; goal D would add C-D. So the
    // optimum is B..C.
    std::vector<bf::SeededEndpoint> sources = {{a, 0.0}, {b, 0.0}};
    std::vector<bf::SeededEndpoint> goals = {{c, 0.0}, {d, 0.0}};
    bf::ShortestPath p = bf::FindShortestPathMulti(g, sources, goals, bf::SearchOptions{});
    REQUIRE(p.found);
    CHECK(p.vertices.front() == b);
    CHECK(p.vertices.back() == c);
    CHECK_THAT(p.distance_nm, WithinRel(60.0, 0.02));
  }

  SECTION("a large seed cost steers the choice to another endpoint") {
    // Make source B expensive so source A (seed 0) wins despite the extra leg;
    // goal D is cheap (seed 0) and goal C is heavily penalized, so the path
    // runs A..D end to end.
    std::vector<bf::SeededEndpoint> sources = {{a, 0.0}, {b, 1000.0}};
    std::vector<bf::SeededEndpoint> goals = {{c, 1000.0}, {d, 0.0}};
    bf::ShortestPath p = bf::FindShortestPathMulti(g, sources, goals, bf::SearchOptions{});
    REQUIRE(p.found);
    CHECK(p.vertices.front() == a);
    CHECK(p.vertices.back() == d);
    // Three enroute steps ~180 NM, both seeds zero.
    CHECK_THAT(p.distance_nm, WithinRel(180.0, 0.02));
  }

  SECTION("seed costs are included in the reported distance") {
    std::vector<bf::SeededEndpoint> sources = {{a, 7.0}};
    std::vector<bf::SeededEndpoint> goals = {{d, 5.0}};
    bf::ShortestPath p = bf::FindShortestPathMulti(g, sources, goals, bf::SearchOptions{});
    REQUIRE(p.found);
    // 180 NM enroute + 7 source seed + 5 goal seed.
    CHECK_THAT(p.distance_nm, WithinRel(192.0, 0.02));
  }
}

TEST_CASE("multi-source/goal A* reports no path when disconnected", "[graph]") {
  bf::NavData d;
  d.waypoints = {bf::Waypoint{bf::Ident("AAA", "ZZ"), bf::Coordinate{0, 0}, {}},
                 bf::Waypoint{bf::Ident("BBB", "ZZ"), bf::Coordinate{10, 10}, {}}};
  bf::GraphBuilder builder(d);  // no airways
  const int a = builder.VertexByIdent("AAA");
  const int b = builder.VertexByIdent("BBB");
  bf::ShortestPath p =
      bf::FindShortestPathMulti(builder.graph(), {{a, 0.0}}, {{b, 0.0}}, bf::SearchOptions{});
  CHECK_FALSE(p.found);
}

}  // namespace
