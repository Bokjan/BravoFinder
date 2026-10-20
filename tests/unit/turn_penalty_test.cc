// SPDX-License-Identifier: MIT
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <utility>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/graph/astar.h"
#include "core/graph/yen_kshortest.h"
#include "io/build/graph_builder.h"

using Catch::Matchers::WithinRel;

// Tests for the turn-angle soft penalty: the penalty model itself,
// and its effect on the A* search. The penalty is path-dependent (it needs the
// inbound heading at a vertex), so it lives in the search loop rather than the
// Constraint machinery; these cover both the pure function and the search
// behavior that the Constraint layer cannot exercise.

namespace {

// A tiny graph of three fixes on the prime meridian, used to pit a short but
// reversing SID exit against a longer but smooth one -- the OC-class shape.
//
//   S1 (lat 1) -- G (lat 0) -- S2 (lat -2)     airway "Q1", both directions
//
// Both sources arrive heading north (bearing 0). From S1 the only way to G is
// south (a 180-degree reversal at the SID exit); from S2 it is north (smooth).
bf::GraphBuilder MakeReversalGraph() {
  bf::NavData d;
  auto wp = [](const char* id, double lat) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{lat, 0.0}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("S1", 1.0), wp("G", 0.0), wp("S2", -2.0)};
  bf::AirwaySegment seg;
  seg.name = "Q1";
  seg.direction = bf::AirwayDirection::kBoth;
  d.airways = {
      {bf::Ident("S1", "ZZ"), bf::Ident("G", "ZZ"), seg},
      {bf::Ident("S2", "ZZ"), bf::Ident("G", "ZZ"), seg},
  };
  auto result = bf::GraphBuilder::Build(d);
  REQUIRE(result);
  return std::move(result).value();
}

}  // namespace

TEST_CASE("TurnAngleDeg normalizes to [0, 180]", "[unit][turn_penalty]") {
  CHECK(bf::TurnAngleDeg(0.0, 0.0) == 0.0);
  CHECK(bf::TurnAngleDeg(0.0, 180.0) == 180.0);
  CHECK(bf::TurnAngleDeg(350.0, 10.0) == 20.0);  // wraps across 0/360
  CHECK(bf::TurnAngleDeg(10.0, 350.0) == 20.0);  // symmetric
  CHECK(bf::TurnAngleDeg(90.0, 270.0) == 180.0);
}

TEST_CASE("TurnPenalty model is zero below the threshold then ramps", "[unit][turn_penalty]") {
  bf::TurnPenalty tp;
  tp.enabled = true;

  SECTION("disabled yields no penalty regardless of angle") {
    tp.enabled = false;
    CHECK(tp(180.0) == 0.0);
    CHECK(tp(90.0) == 0.0);
  }

  SECTION("zero up to the 45-degree threshold") {
    CHECK(tp(0.0) == 0.0);
    CHECK(tp(45.0) == 0.0);
  }

  SECTION("linear from threshold to the 90-degree knee") {
    // 45 -> 0, 90 -> 20 NM (kPenaltyAtKneeNm), linear between.
    CHECK_THAT(tp(90.0), WithinRel(bf::TurnPenalty::kPenaltyAtKneeNm, 1e-9));
    const double mid = tp(67.5);  // halfway in angle
    CHECK_THAT(mid, WithinRel(bf::TurnPenalty::kPenaltyAtKneeNm / 2.0, 1e-9));
  }

  SECTION("quadratic from the knee to 180") {
    CHECK_THAT(tp(180.0), WithinRel(bf::TurnPenalty::kMaxPenaltyNm, 1e-9));
    // 157 deg (the OC case): t=(157-90)/90=0.7444 -> 20 + 180*0.7444^2 ~= 119.8.
    CHECK_THAT(tp(157.0), WithinRel(119.76, 0.01));
    // Quadratic means the 135-degree midpoint is below the linear midpoint.
    const double q135 = tp(135.0);  // t = 0.5 -> 20 + 180*0.25 = 65
    CHECK_THAT(q135, WithinRel(65.0, 1e-9));
    CHECK(q135 < bf::TurnPenalty::kMaxPenaltyNm / 2.0 + 1.0);
  }

  SECTION("monotonic non-decreasing across the whole range") {
    double prev = -1.0;
    for (int a = 0; a <= 180; ++a) {
      const double v = tp(static_cast<double>(a));
      CHECK(v >= prev);
      prev = v;
    }
  }
}

TEST_CASE("Coordinate::BearingTo matches cardinal directions", "[unit][turn_penalty]") {
  using bf::Coordinate;
  const Coordinate o{0.0, 0.0};
  CHECK_THAT(o.BearingTo({0.0, 1.0}), WithinRel(90.0, 1e-6));    // east
  CHECK_THAT(o.BearingTo({0.0, -1.0}), WithinRel(270.0, 1e-6));  // west
  CHECK_THAT(o.BearingTo({1.0, 0.0}), WithinRel(0.0, 1e-6));     // north
  CHECK_THAT(o.BearingTo({-1.0, 0.0}), WithinRel(180.0, 1e-6));  // south
}

TEST_CASE("A* turn penalty steers off a reversing SID exit", "[unit][turn_penalty]") {
  bf::GraphBuilder builder = MakeReversalGraph();
  const int s1 = builder.VerticesByIdent("S1")[0];
  const int s2 = builder.VerticesByIdent("S2")[0];
  const int g = builder.VerticesByIdent("G")[0];
  const bf::NavGraph& graph = builder.graph();

  // Both sources arrive heading north (bearing 0). S1 is 60 NM north of G, so
  // reaching G reverses to south (180-degree turn); S2 is 120 NM south of G, a
  // smooth northbound join. S1 is the shorter route, S2 the smoother one.
  const std::vector<bf::SeededEndpoint> sources = {
      {s1, 0.0, 0.0},  // short but reverses onto G
      {s2, 0.0, 0.0},  // long but smooth
  };
  const std::vector<bf::SeededEndpoint> goals = {{g, 0.0, -1.0}};  // no STAR heading

  SECTION("with the penalty disabled, the shorter (reversing) source wins") {
    bf::SearchOptions opts;
    opts.turn_penalty.enabled = false;
    bf::ShortestPath p = bf::FindShortestPathMulti(graph, sources, goals, opts);
    REQUIRE(p.found);
    CHECK(p.vertices.front() == s1);  // 60 NM beats 120 NM
    CHECK(p.vertices.back() == g);
  }

  SECTION("with the penalty enabled, the smooth (longer) source wins") {
    bf::SearchOptions opts;
    opts.turn_penalty.enabled = true;
    bf::ShortestPath p = bf::FindShortestPathMulti(graph, sources, goals, opts);
    REQUIRE(p.found);
    CHECK(p.vertices.front() == s2);  // 120 NM + 0 penalty beats 60 NM + 200 penalty
    CHECK(p.vertices.back() == g);
    // Geographic distance is the smooth route's ~120 NM. The smooth route turns
    // 0 degrees, so it accrues no penalty: cost equals distance. The penalty
    // lives only in cost, never in distance.
    CHECK_THAT(p.distance_nm, WithinRel(120.0, 0.5));
    CHECK_THAT(p.cost, WithinRel(p.distance_nm, 1e-6));
  }

  SECTION("the penalty never blocks a route -- a single reversing source still resolves") {
    bf::SearchOptions opts;
    opts.turn_penalty.enabled = true;
    bf::ShortestPath p =
        bf::FindShortestPathMulti(graph, std::vector<bf::SeededEndpoint>{{s1, 0.0, 0.0}},
                                  std::vector<bf::SeededEndpoint>{{g, 0.0, -1.0}}, opts);
    REQUIRE(p.found);
    CHECK(p.vertices.front() == s1);
    // The 180-degree reversal at S1 is priced into cost but the route exists.
    CHECK(p.cost > p.distance_nm);
  }
}

namespace {

// A source with no heading, two ways to G (all airways forward-only):
//
//   S (0,0) -> A (0,1) -> G (0,0.5)      90 NM, reverses 180 degrees at A
//   S (0,0) -> C (1.2,0.25) -> G         ~147 NM, a ~156-degree turn at C
//
// S-A-G is shorter but its reversal at A is the dearer turn.
bf::GraphBuilder MakeMidPathReversalGraph() {
  bf::NavData d;
  auto wp = [](const char* id, double lat, double lon) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{lat, lon}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("S", 0.0, 0.0), wp("A", 0.0, 1.0), wp("G", 0.0, 0.5), wp("C", 1.2, 0.25)};
  bf::AirwaySegment seg;
  seg.name = "Q1";
  seg.direction = bf::AirwayDirection::kForward;
  d.airways = {
      {bf::Ident("S", "ZZ"), bf::Ident("A", "ZZ"), seg},
      {bf::Ident("A", "ZZ"), bf::Ident("G", "ZZ"), seg},
      {bf::Ident("S", "ZZ"), bf::Ident("C", "ZZ"), seg},
      {bf::Ident("C", "ZZ"), bf::Ident("G", "ZZ"), seg},
  };
  auto result = bf::GraphBuilder::Build(d);
  REQUIRE(result);
  return std::move(result).value();
}

}  // namespace

TEST_CASE("A* prices mid-path turns when the source has no heading", "[unit][turn_penalty]") {
  // A forced via-point seeds the next leg with no heading. Only the first edge
  // out of it is free to choose; the turn at A must still be priced, or the
  // search walks straight into the reversal it exists to avoid.
  bf::GraphBuilder builder = MakeMidPathReversalGraph();
  const bf::NavGraph& graph = builder.graph();
  const int s = builder.VerticesByIdent("S")[0];
  const int c = builder.VerticesByIdent("C")[0];
  const int g = builder.VerticesByIdent("G")[0];
  bf::SearchOptions opts;
  opts.turn_penalty.enabled = true;
  const std::vector<bf::SeededEndpoint> sources = {{s, 0.0, bf::kNoBearing}};
  const std::vector<bf::SeededEndpoint> goals = {{g, 0.0, bf::kNoBearing}};

  const bf::ShortestPath p = bf::FindShortestPathMulti(graph, sources, goals, opts);
  REQUIRE(p.found);
  CHECK(p.vertices == std::vector<int>{s, c, g});

  // The search's own cost matches the independent re-cost of the same path.
  const int n = graph.VertexCount();
  std::vector<double> source_seed(n, -1.0);
  std::vector<double> goal_seed(n, -1.0);
  source_seed[s] = 0.0;
  goal_seed[g] = 0.0;
  const std::vector<double> no_bearing(n, bf::kNoBearing);
  double cost = 0.0;
  double distance = 0.0;
  REQUIRE(bf::CostOfPathMulti(graph, p.vertices, source_seed, goal_seed, no_bearing, no_bearing,
                              opts, cost, distance));
  CHECK_THAT(p.cost, WithinRel(cost, 1e-9));
}

TEST_CASE("Yen prices a source-is-goal path like A* does", "[unit][turn_penalty]") {
  // V is both a source (arriving north, bearing 0) and the goal (leaving
  // south, bearing 180), so the empty path at V still turns 180 degrees. U is
  // the other source, 60 NM south of V and also arriving north.
  bf::NavData d;
  auto wp = [](const char* id, double lat) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{lat, 0.0}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("V", 0.0), wp("W", 1.0), wp("U", -1.0)};
  bf::AirwaySegment seg;
  seg.name = "Q1";
  seg.direction = bf::AirwayDirection::kForward;
  d.airways = {{bf::Ident("V", "ZZ"), bf::Ident("W", "ZZ"), seg},
               {bf::Ident("U", "ZZ"), bf::Ident("V", "ZZ"), seg}};
  auto built = bf::GraphBuilder::Build(d);
  REQUIRE(built);
  const bf::GraphBuilder builder = std::move(built).value();
  const int v = builder.VerticesByIdent("V")[0];
  const int u = builder.VerticesByIdent("U")[0];
  bf::SearchOptions opts;
  opts.turn_penalty.enabled = true;

  // U-V costs 60 NM + the 180-degree turn at V; V alone costs its 100 NM seed
  // + the same turn. So U-V comes first, and V must come second at ~300, not
  // at 100 with the turn dropped (which would rank it below the first path).
  const std::vector<bf::ShortestPath> paths = bf::FindKShortestPathsMulti(
      builder.graph(), {{v, 100.0, 0.0}, {u, 0.0, 0.0}}, {{v, 0.0, 180.0}}, 2, opts);
  REQUIRE(paths.size() == 2);
  CHECK(paths[0].vertices == std::vector<int>{u, v});
  CHECK(paths[1].vertices == std::vector<int>{v});
  CHECK(paths[0].cost <= paths[1].cost);
  CHECK_THAT(paths[1].cost, WithinRel(100.0 + bf::TurnPenalty::kMaxPenaltyNm, 1e-9));
}
