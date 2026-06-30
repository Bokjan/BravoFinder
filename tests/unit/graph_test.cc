#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "core/graph/astar.h"
#include "io/graph_builder.h"

using Catch::Matchers::WithinRel;

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

}  // namespace
