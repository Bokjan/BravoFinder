// SPDX-License-Identifier: MIT
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "core/constraints/altitude_constraints.h"
#include "core/graph/astar.h"
#include "core/routing/route_request.h"
#include "io/graph_builder.h"

using Catch::Matchers::WithinRel;

TEST_CASE("GraphEdge stays compact (16 bytes)", "[unit][graph]") {
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

TEST_CASE("A* finds path along a linear airway", "[unit][graph]") {
  bf::GraphBuilder builder(MakeLineData());
  const int a = builder.VerticesByIdent("AAA")[0];
  const int d = builder.VerticesByIdent("DDD")[0];
  REQUIRE(a >= 0);
  REQUIRE(d >= 0);

  bf::ShortestPath path = bf::FindShortestPath(builder.graph(), a, d);
  REQUIRE(path.found);
  // A-B-C-D = 4 vertices.
  CHECK(path.vertices.size() == 4);
  // Three degrees of longitude at the equator is ~180 NM.
  CHECK_THAT(path.distance_nm, WithinRel(180.0, 0.01));
}

TEST_CASE("one-way airway is not traversable backward", "[unit][graph]") {
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
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];

  // Forward is allowed...
  CHECK(bf::FindShortestPath(builder.graph(), a, b).found);
  // ...backward is not.
  CHECK_FALSE(bf::FindShortestPath(builder.graph(), b, a).found);
}

TEST_CASE("a forward-only airway leaves its destination inbound-only", "[unit][graph]") {
  // Mirrors the real ABBEY topology: a forward-only airway A -> B gives B an
  // inbound edge but no outbound. B is on-network (union) and usable as a STAR
  // entry gate (inbound), but not as a SID hand-off (outbound). A is the reverse.
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
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];

  CHECK(builder.HasOutbound(a));
  CHECK_FALSE(builder.HasInbound(a));
  CHECK(builder.HasInbound(b));
  CHECK_FALSE(builder.HasOutbound(b));
  // Both touch the network, so the display-facing union is true for each.
  CHECK(builder.OnNetwork(a));
  CHECK(builder.OnNetwork(b));
}

TEST_CASE("unreachable vertices report no path", "[unit][graph]") {
  bf::NavData d;
  d.waypoints = {bf::Waypoint{bf::Ident("AAA", "ZZ"), bf::Coordinate{0, 0}, {}},
                 bf::Waypoint{bf::Ident("BBB", "ZZ"), bf::Coordinate{10, 10}, {}}};
  // No airways: the two points are disconnected.
  bf::GraphBuilder builder(d);
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];
  CHECK_FALSE(bf::FindShortestPath(builder.graph(), a, b).found);
}

TEST_CASE("multi-source/goal A* picks the cheapest seeded combination", "[unit][graph]") {
  // A-B-C-D in a line (~60 NM per degree-step at the equator). Sources seed at
  // A and B; goals seed at C and D. The search must weigh seed costs against
  // enroute distance to choose the best end-to-end combination.
  bf::GraphBuilder builder(MakeLineData());
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];
  const int c = builder.VerticesByIdent("CCC")[0];
  const int d = builder.VerticesByIdent("DDD")[0];
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

TEST_CASE("multi-source/goal A* reports no path when disconnected", "[unit][graph]") {
  bf::NavData d;
  d.waypoints = {bf::Waypoint{bf::Ident("AAA", "ZZ"), bf::Coordinate{0, 0}, {}},
                 bf::Waypoint{bf::Ident("BBB", "ZZ"), bf::Coordinate{10, 10}, {}}};
  bf::GraphBuilder builder(d);  // no airways
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];
  bf::ShortestPath p =
      bf::FindShortestPathMulti(builder.graph(), {{a, 0.0}}, {{b, 0.0}}, bf::SearchOptions{});
  CHECK_FALSE(p.found);
}

TEST_CASE("VerticesByIdent returns every region match for a reused ident", "[unit][graph]") {
  // An ident is reused across regions (e.g. the same fix code in K6 and EH).
  // The builder must surface ALL matches, never silently pick one.
  bf::NavData d;
  d.waypoints = {
      bf::Waypoint{bf::Ident("SHARED", "K6"), bf::Coordinate{50.0, 4.0}, bf::WaypointKind::kFix},
      bf::Waypoint{bf::Ident("SHARED", "EH"), bf::Coordinate{52.0, 4.5}, bf::WaypointKind::kFix},
      bf::Waypoint{bf::Ident("SHARED", "LF"), bf::Coordinate{48.0, 3.0}, bf::WaypointKind::kFix},
      bf::Waypoint{bf::Ident("LONE", "K6"), bf::Coordinate{49.0, 2.0}, bf::WaypointKind::kFix},
  };
  bf::GraphBuilder builder(d);

  // The reused ident resolves to all three regions, in insertion order.
  const std::vector<int> shared = builder.VerticesByIdent("SHARED");
  REQUIRE(shared.size() == 3);
  CHECK(builder.IdentOf(shared[0]).region == "K6");
  CHECK(builder.IdentOf(shared[1]).region == "EH");
  CHECK(builder.IdentOf(shared[2]).region == "LF");

  // A unique ident still resolves to exactly one vertex.
  const std::vector<int> lone = builder.VerticesByIdent("LONE");
  REQUIRE(lone.size() == 1);
  CHECK(builder.IdentOf(lone[0]).region == "K6");

  // An unknown ident resolves to nothing.
  CHECK(builder.VerticesByIdent("NOPE").empty());
}

// Two parallel airways A->B, one low and one high, share both endpoints and so
// have identical geographic distance. SelectEdge must return the edge the search
// would have relaxed cheapest -- with a high-level preference, the HIGH edge --
// not whichever parallel edge happens to come first in the edge list. This is
// the guarantee MakeRoute relies on to label a leg's via/airway consistently
// with the path's cost model (regression for M5).
TEST_CASE("SelectEdge returns the cost-model edge among parallel airways", "[unit][graph]") {
  bf::NavData d;
  auto wp = [](const char* id, double lon) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{0.0, lon}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("AAA", 0.0), wp("BBB", 1.0)};
  auto seg = [](const char* name, bf::AirwayLevel level) {
    bf::AirwaySegment s;
    s.name = name;
    s.direction = bf::AirwayDirection::kBoth;
    s.level = level;
    return s;
  };
  // Low airway V1 listed first, high airway J1 second.
  d.airways = {
      {bf::Ident("AAA", "ZZ"), bf::Ident("BBB", "ZZ"), seg("V1", bf::AirwayLevel::kLow)},
      {bf::Ident("AAA", "ZZ"), bf::Ident("BBB", "ZZ"), seg("J1", bf::AirwayLevel::kHigh)},
  };
  bf::GraphBuilder builder(d);
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];
  REQUIRE(a >= 0);
  REQUIRE(b >= 0);

  // Two parallel edges must exist (the builder keeps concurrent airways).
  int parallel = 0;
  for (const bf::GraphEdge* e = builder.graph().EdgesBegin(a); e != builder.graph().EdgesEnd(a);
       ++e) {
    if (e->to == b) {
      ++parallel;
    }
  }
  REQUIRE(parallel == 2);

  // No preference: any allowed edge is fine, but one must be returned.
  const bf::GraphEdge* any = bf::SelectEdge(builder.graph(), a, b, bf::SearchOptions{});
  REQUIRE(any != nullptr);

  // High-level preference: the low edge is penalized, so the high edge is cheaper
  // by effective cost and must be the one SelectEdge returns.
  bf::RouteRequest req;
  req.level = bf::LevelPreference::kHigh;
  const bf::LevelPreferenceConstraint level_pref;
  bf::SearchOptions opts;
  opts.request = &req;
  opts.constraints = {&level_pref};
  const bf::GraphEdge* hi = bf::SelectEdge(builder.graph(), a, b, opts);
  REQUIRE(hi != nullptr);
  CHECK(hi->level == bf::AirwayLevel::kHigh);
  CHECK(builder.AirwayName(hi->airway_id) == "J1");

  // Symmetrically, a low-level preference selects the low edge.
  req.level = bf::LevelPreference::kLow;
  const bf::GraphEdge* lo = bf::SelectEdge(builder.graph(), a, b, opts);
  REQUIRE(lo != nullptr);
  CHECK(lo->level == bf::AirwayLevel::kLow);
  CHECK(builder.AirwayName(lo->airway_id) == "V1");
}

TEST_CASE("constraints with a null request refuse edges, not UB", "[unit][graph]") {
  // SearchOptions contract: request must be set when constraints are present.
  // A null request must not dereference null in release (NDEBUG) builds; the
  // guard refuses the edge so the search reports no path rather than crashing.
  // Regression for M8: the old bare assert let release dereference null here.
  bf::GraphBuilder builder(MakeLineData());
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];
  const int d = builder.VerticesByIdent("DDD")[0];

  const bf::LevelPreferenceConstraint level_pref;
  bf::SearchOptions opts;
  opts.constraints = {&level_pref};  // request intentionally left null

  // SelectEdge returns nullptr (every edge refused) instead of dereferencing null.
  CHECK(bf::SelectEdge(builder.graph(), a, b, opts) == nullptr);
  // A full search reports no path through the refused edges.
  CHECK_FALSE(bf::FindShortestPath(builder.graph(), a, d, opts).found);
}

}  // namespace
