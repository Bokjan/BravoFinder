// SPDX-License-Identifier: MIT
#include "io/navdb/forced_router.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <random>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/graph/astar.h"
#include "io/build/graph_builder.h"

namespace {

bf::GraphBuilder MakeBuilder(const bf::NavData& data) {
  auto result = bf::GraphBuilder::Build(data);
  REQUIRE(result);
  return std::move(result).value();
}

bf::Waypoint Wp(const std::string& id, double lat, double lon) {
  return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{lat, lon}, bf::WaypointKind::kFix};
}

bf::AirwaySegment TwoWay(const std::string& name) {
  bf::AirwaySegment seg;
  seg.name = name;
  seg.direction = bf::AirwayDirection::kBoth;
  return seg;
}

}  // namespace

TEST_CASE("forced routing survives the cheapest hops sharing a fix", "[unit][forced]") {
  // Via F. The only S -> F path runs through X, and the cheapest F -> G path
  // runs back through X too, so the all-best combination repeats X. The route
  // S X F Y G exists and must be found whatever k is.
  //
  //            F (1,1) --- Y (1.6,2.3)
  //            |              |
  //   S (0,0)--X (0,1) ------ G (0,2)
  bf::NavData d;
  d.waypoints = {Wp("S", 0.0, 0.0), Wp("X", 0.0, 1.0), Wp("F", 1.0, 1.0), Wp("G", 0.0, 2.0),
                 Wp("Y", 1.6, 2.3)};
  d.airways = {{bf::Ident("S", "ZZ"), bf::Ident("X", "ZZ"), TwoWay("Q1")},
               {bf::Ident("X", "ZZ"), bf::Ident("F", "ZZ"), TwoWay("Q2")},
               {bf::Ident("X", "ZZ"), bf::Ident("G", "ZZ"), TwoWay("Q1")},
               {bf::Ident("F", "ZZ"), bf::Ident("Y", "ZZ"), TwoWay("Q3")},
               {bf::Ident("Y", "ZZ"), bf::Ident("G", "ZZ"), TwoWay("Q3")}};
  const bf::GraphBuilder builder = MakeBuilder(d);
  const auto vertex = [&builder](const char* id) { return builder.VerticesByIdent(id)[0]; };
  const std::vector<int> want = {vertex("S"), vertex("X"), vertex("F"), vertex("Y"), vertex("G")};

  for (int k : {1, 2, 5}) {
    CAPTURE(k);
    const std::vector<bf::ShortestPath> paths =
        bf::ForcedRouter::FindPaths(builder.graph(), {{vertex("S"), 0.0}}, {{vertex("G"), 0.0}},
                                    {vertex("F")}, k, bf::SearchOptions{});
    REQUIRE_FALSE(paths.empty());
    CHECK(paths.front().vertices == want);
  }
}

TEST_CASE("forced routes come back simple, in order, and cheapest first", "[unit][forced]") {
  // Random small networks with the turn penalty on, so the turns at the forced
  // fixes -- which no single hop sees -- reorder the stitched routes.
  std::mt19937 rng(0xF0A7);  // fixed seed: reproducible
  int checked = 0;
  for (int trial = 0; trial < 150; ++trial) {
    const int n = std::uniform_int_distribution<int>(6, 14)(rng);
    std::uniform_real_distribution<double> lat(-2.0, 2.0);
    std::uniform_real_distribution<double> lon(0.0, 6.0);
    bf::NavData d;
    for (int i = 0; i < n; ++i) {
      d.waypoints.push_back(Wp("W" + std::to_string(i), lat(rng), lon(rng)));
    }
    std::uniform_int_distribution<int> pick(0, n - 1);
    for (int i = 0; i + 1 < n; ++i) {
      d.airways.push_back({d.waypoints[i].ident, d.waypoints[i + 1].ident, TwoWay("R")});
    }
    for (int e = 0; e < 2 * n; ++e) {
      const int a = pick(rng);
      const int b = pick(rng);
      if (a != b) {
        d.airways.push_back({d.waypoints[a].ident, d.waypoints[b].ident, TwoWay("R")});
      }
    }
    const bf::GraphBuilder builder = MakeBuilder(d);
    const auto vertex = [&builder](int i) {
      return builder.VerticesByIdent("W" + std::to_string(i))[0];
    };
    std::vector<int> ends;
    while (ends.size() < 3) {
      const int v = vertex(pick(rng));
      if (std::find(ends.begin(), ends.end(), v) == ends.end()) {
        ends.push_back(v);
      }
    }
    bf::SearchOptions opts;
    opts.turn_penalty.enabled = true;
    const int k = 1 + trial % 6;
    const std::vector<bf::ShortestPath> paths = bf::ForcedRouter::FindPaths(
        builder.graph(), {{ends[0], 0.0, 90.0}}, {{ends[2], 0.0}}, {ends[1]}, k, opts);
    CAPTURE(trial, k);
    CHECK(static_cast<int>(paths.size()) <= k);
    for (size_t i = 0; i < paths.size(); ++i) {
      const std::vector<int>& p = paths[i].vertices;
      REQUIRE_FALSE(p.empty());
      CHECK(p.front() == ends[0]);
      CHECK(p.back() == ends[2]);
      CHECK(std::find(p.begin(), p.end(), ends[1]) != p.end());
      CHECK(std::unordered_set<int>(p.begin(), p.end()).size() == p.size());
      if (i > 0) {
        CHECK(paths[i - 1].cost <= paths[i].cost);
      }
    }
    if (!paths.empty()) {
      ++checked;
    }
  }
  CHECK(checked > 100);
}
