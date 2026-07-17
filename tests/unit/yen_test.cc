#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "core/graph/astar.h"
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
  const int a = builder.VerticesByIdent("AAA")[0];
  const int dd = builder.VerticesByIdent("DDD")[0];
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
  const int bbb = builder.VerticesByIdent("BBB")[0];
  CHECK(paths[0].vertices[1] == bbb);
}

TEST_CASE("Yen with k=1 returns just the best path", "[yen]") {
  bf::GraphBuilder builder(MakeDiamondData());
  const int a = builder.VerticesByIdent("AAA")[0];
  const int dd = builder.VerticesByIdent("DDD")[0];
  std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPaths(builder.graph(), a, dd, 1, bf::SearchOptions{});
  REQUIRE(paths.size() == 1);
}

TEST_CASE("Yen on unreachable goal returns empty", "[yen]") {
  bf::NavData d;
  d.waypoints = {bf::Waypoint{bf::Ident("AAA", "ZZ"), bf::Coordinate{0, 0}, {}},
                 bf::Waypoint{bf::Ident("BBB", "ZZ"), bf::Coordinate{0, 5}, {}}};
  bf::GraphBuilder builder(d);
  const int a = builder.VerticesByIdent("AAA")[0];
  const int b = builder.VerticesByIdent("BBB")[0];
  std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPaths(builder.graph(), a, b, 3, bf::SearchOptions{});
  CHECK(paths.empty());
}

// A network with two distinct entry fixes (S1, S2) that both lead to the same
// goal G through a shared midpoint M. Each entry sits one degree out on either
// side, so a route can join through either one at nearly equal cost.
//
//   S1 --Q1--
//             M --Q3--> G
//   S2 --Q2--/
//
bf::NavData MakeTwoEntryData() {
  bf::NavData d;
  auto wp = [](const char* id, double lat, double lon) {
    return bf::Waypoint{bf::Ident(id, "ZZ"), bf::Coordinate{lat, lon}, bf::WaypointKind::kFix};
  };
  d.waypoints = {wp("S1", 0.5, 0.0), wp("S2", -0.5, 0.0), wp("MMM", 0.0, 1.0), wp("GGG", 0.0, 2.0)};
  auto seg = [](const char* name) {
    bf::AirwaySegment s;
    s.name = name;
    s.direction = bf::AirwayDirection::kBoth;
    return s;
  };
  d.airways = {
      {bf::Ident("S1", "ZZ"), bf::Ident("MMM", "ZZ"), seg("Q1")},
      {bf::Ident("S2", "ZZ"), bf::Ident("MMM", "ZZ"), seg("Q2")},
      {bf::Ident("MMM", "ZZ"), bf::Ident("GGG", "ZZ"), seg("Q3")},
  };
  return d;
}

TEST_CASE("multi-endpoint Yen yields candidates through different entry fixes", "[yen]") {
  bf::GraphBuilder builder(MakeTwoEntryData());
  const int s1 = builder.VerticesByIdent("S1")[0];
  const int s2 = builder.VerticesByIdent("S2")[0];
  const int g = builder.VerticesByIdent("GGG")[0];
  REQUIRE(s1 >= 0);
  REQUIRE(s2 >= 0);
  REQUIRE(g >= 0);

  // Two seeded source fixes (the two procedure entry fixes), one goal. The seeds
  // are equal, so the only difference between the two cheapest routes is which
  // entry fix they join through -- exactly the cross-fix alternative the earlier
  // single-fix scheme could not produce.
  const std::vector<bf::SeededEndpoint> sources = {{s1, 10.0}, {s2, 10.0}};
  const std::vector<bf::SeededEndpoint> goals = {{g, 0.0}};

  std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPathsMulti(builder.graph(), sources, goals, 3, bf::SearchOptions{});

  REQUIRE(paths.size() == 2);
  CHECK(paths[0].cost <= paths[1].cost);
  // The two candidates start at different entry fixes.
  CHECK(paths[0].vertices.front() != paths[1].vertices.front());
  std::set<int> entries = {paths[0].vertices.front(), paths[1].vertices.front()};
  CHECK(entries == std::set<int>{s1, s2});
  // Both include the shared source seed in their reported distance.
  CHECK(paths[0].distance_nm >= 10.0);
}

// A layered lattice with many genuinely distinct source->goal paths, used to
// pin down the FULL candidate sequence (cost order + vertex lists) that Yen
// produces. This is the golden-output regression guard for the Lawler
// optimization: Lawler only skips redundant spur computations, so it must leave
// the accepted paths and their order byte-for-byte identical. A lattice with
// several columns of parallel nodes gives long paths with deviations at many
// positions, which is exactly what exercises Lawler's per-path deviation index.
//
//   col 0      col 1      col 2      col 3
//   L0r0 ---- L1r0 ---- L2r0 ---- L3r0
//        \  X      \  X      \  X
//   L0r1 ---- L1r1 ---- L2r1 ---- L3r1
//
// Every node in column c connects to every node in column c+1, so there are
// many paths whose costs differ slightly (rows sit at different latitudes).
bf::NavData MakeLatticeData(int cols, int rows) {
  bf::NavData d;
  auto seg = [](const std::string& name) {
    bf::AirwaySegment s;
    s.name = name;
    s.direction = bf::AirwayDirection::kBoth;
    return s;
  };
  auto name = [](int c, int r) { return "L" + std::to_string(c) + "r" + std::to_string(r); };
  for (int c = 0; c < cols; ++c) {
    for (int r = 0; r < rows; ++r) {
      // Rows fan out in latitude so parallel paths have distinct lengths.
      const double lat = (r - (rows - 1) / 2.0) * 0.2;
      d.waypoints.push_back(bf::Waypoint{bf::Ident(name(c, r), "ZZ"),
                                         bf::Coordinate{lat, static_cast<double>(c)},
                                         bf::WaypointKind::kFix});
    }
  }
  for (int c = 0; c + 1 < cols; ++c) {
    for (int r = 0; r < rows; ++r) {
      for (int r2 = 0; r2 < rows; ++r2) {
        d.airways.push_back({bf::Ident(name(c, r), "ZZ"), bf::Ident(name(c + 1, r2), "ZZ"),
                             seg("A" + std::to_string(c))});
      }
    }
  }
  return d;
}

// Serialize a candidate list to a comparable signature: cost (rounded) + the
// vertex sequence, one path per line. Distinct paths that tie on cost still
// differ here, so this catches any reordering or set change.
std::string Signature(const std::vector<bf::ShortestPath>& paths) {
  std::string sig;
  for (const bf::ShortestPath& p : paths) {
    sig += "cost=" + std::to_string(static_cast<long long>(p.cost * 1e6)) + " [";
    for (int v : p.vertices) {
      sig += std::to_string(v) + ",";
    }
    sig += "]\n";
  }
  return sig;
}

TEST_CASE("Yen golden candidate sequence on a lattice (Lawler regression guard)", "[yen]") {
  // A 4-column, 3-row lattice: enough parallel paths that k=8 exercises spur
  // deviations at every position. If this signature changes, the k-shortest
  // result set or its order changed -- which the Lawler optimization must NOT do.
  bf::GraphBuilder builder(MakeLatticeData(/*cols=*/4, /*rows=*/3));
  const int start = builder.VerticesByIdent("L0r1")[0];
  const int goal = builder.VerticesByIdent("L3r1")[0];
  REQUIRE(start >= 0);
  REQUIRE(goal >= 0);

  const std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPaths(builder.graph(), start, goal, 8, bf::SearchOptions{});

  // Sanity: strictly cost-ordered, all loopless and distinct.
  REQUIRE(paths.size() >= 5);
  std::set<std::vector<int>> seen;
  for (size_t i = 0; i < paths.size(); ++i) {
    CHECK(paths[i].found);
    if (i > 0) {
      CHECK(paths[i - 1].cost <= paths[i].cost);
    }
    CHECK(seen.insert(paths[i].vertices).second);  // no duplicate path
  }

  // Golden signature captured from the current (pre-Lawler) implementation. The
  // Lawler change must reproduce this exactly (it only skips redundant spur
  // computations; the accepted set and order are unchanged). The vertex ids are
  // positional in MakeLatticeData, so renumbering its construction rewrites the
  // whole signature.
  static const std::string kGolden =
      "cost=180121616 [1,4,7,10,]\n"
      "cost=182499088 [1,3,6,10,]\n"
      "cost=182499088 [1,5,8,10,]\n"
      "cost=182499454 [1,3,7,10,]\n"
      "cost=182499454 [1,4,6,10,]\n"
      "cost=182499454 [1,4,8,10,]\n"
      "cost=182499454 [1,5,7,10,]\n"
      "cost=187124443 [1,3,8,10,]\n";
  CHECK(Signature(paths) == kGolden);
}

TEST_CASE("multi-source Yen golden sequence on a lattice (Lawler regression guard)", "[yen]") {
  // Same lattice, but as a multi-source/multi-goal search with two seeded entry
  // columns and two seeded goal rows -- exercising the super-source spur (index
  // -1) path that Lawler's deviation index must represent.
  bf::GraphBuilder builder(MakeLatticeData(/*cols=*/4, /*rows=*/3));
  const std::vector<bf::SeededEndpoint> sources = {{builder.VerticesByIdent("L0r0")[0], 5.0},
                                                   {builder.VerticesByIdent("L0r1")[0], 5.0}};
  const std::vector<bf::SeededEndpoint> goals = {{builder.VerticesByIdent("L3r0")[0], 3.0},
                                                 {builder.VerticesByIdent("L3r1")[0], 3.0}};
  for (const auto& s : sources) REQUIRE(s.vertex >= 0);
  for (const auto& g : goals) REQUIRE(g.vertex >= 0);

  const std::vector<bf::ShortestPath> paths =
      bf::FindKShortestPathsMulti(builder.graph(), sources, goals, 8, bf::SearchOptions{});

  REQUIRE(paths.size() >= 5);
  std::set<std::vector<int>> seen;
  for (size_t i = 0; i < paths.size(); ++i) {
    CHECK(paths[i].found);
    if (i > 0) {
      CHECK(paths[i - 1].cost <= paths[i].cost);
    }
    CHECK(seen.insert(paths[i].vertices).second);
  }

  // Golden signature (positional vertex ids; regenerate if MakeLatticeData
  // changes its numbering).
  static const std::string kGolden =
      "cost=188120517 [0,3,6,9,]\n"
      "cost=188121616 [1,4,7,10,]\n"
      "cost=189309803 [0,3,6,10,]\n"
      "cost=189309803 [1,3,6,9,]\n"
      "cost=189310169 [0,3,7,10,]\n"
      "cost=189310169 [1,4,6,9,]\n"
      "cost=189310535 [0,4,7,10,]\n"
      "cost=189310535 [1,4,7,9,]\n";
  CHECK(Signature(paths) == kGolden);
}

// --- Differential (golden-reference) testing of the Lawler optimization. ---
//
// The two reference implementations below are deliberately NAIVE: they spur from
// index 0 (single-source) or -1 (multi-source) on every round, with no deviation
// tracking. This is textbook Yen WITHOUT Lawler's optimization. Lawler only skips
// spur computations that would regenerate already-considered paths, so the two
// must produce byte-identical accepted sequences. Running both on hundreds of
// random graphs cross-checks the optimized code against an independent, simpler
// implementation -- far broader coverage than a fixed golden signature.
//
// Both share the SAME candidate ordering as production (cost, then vertices), so
// tie-breaks match exactly.

// A candidate in the naive reference, ordered by (cost, vertices) like production.
struct RefCandidate {
  double cost;
  double distance;
  std::vector<int> vertices;
  bool operator<(const RefCandidate& o) const {
    if (cost != o.cost) {
      return cost < o.cost;
    }
    return vertices < o.vertices;
  }
};

// Effective cost + geographic distance of a fully specified path; false if any
// step is not a real edge. Mirrors the production CostOfPath (no constraints in
// these tests, so no soft penalties).
bool RefCostOfPath(const bf::NavGraph& g, const std::vector<int>& path, double& cost,
                   double& dist) {
  cost = 0.0;
  dist = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const bf::GraphEdge* found = nullptr;
    for (const bf::GraphEdge* e = g.EdgesBegin(path[i]); e != g.EdgesEnd(path[i]); ++e) {
      if (e->to == path[i + 1]) {
        found = e;
        break;
      }
    }
    if (found == nullptr) {
      return false;
    }
    cost += found->distance_nm;
    dist += found->distance_nm;
  }
  return true;
}

// Naive single-source Yen (no Lawler): spur from index 0 every round.
std::vector<bf::ShortestPath> NaiveKShortest(const bf::NavGraph& graph, int start, int goal,
                                             int k) {
  std::vector<bf::ShortestPath> result;
  if (k <= 0) {
    return result;
  }
  bf::ShortestPath first = bf::FindShortestPath(graph, start, goal, bf::SearchOptions{});
  if (!first.found) {
    return result;
  }
  result.push_back(std::move(first));
  std::set<RefCandidate> candidates;
  for (int kth = 1; kth < k; ++kth) {
    const std::vector<int> prev = result.back().vertices;
    for (size_t i = 0; i + 1 < prev.size(); ++i) {
      const std::vector<int> root(prev.begin(), prev.begin() + i + 1);
      std::set<std::pair<int, int>> banned_edges;
      for (const bf::ShortestPath& p : result) {
        if (p.vertices.size() > i + 1 && std::equal(root.begin(), root.end(), p.vertices.begin())) {
          banned_edges.emplace(p.vertices[i], p.vertices[i + 1]);
        }
      }
      const std::set<int> banned_nodes(root.begin(), root.end() - 1);
      bf::SearchOptions opts;
      opts.node_blocked = [banned_nodes](int v) { return banned_nodes.count(v) != 0; };
      opts.edge_blocked = [banned_edges](int f, int t) { return banned_edges.count({f, t}) != 0; };
      const bf::ShortestPath spur = bf::FindShortestPath(graph, prev[i], goal, opts);
      if (!spur.found) {
        continue;
      }
      std::vector<int> total(root.begin(), root.end() - 1);
      total.insert(total.end(), spur.vertices.begin(), spur.vertices.end());
      double cost = 0.0, dist = 0.0;
      if (RefCostOfPath(graph, total, cost, dist)) {
        candidates.insert(RefCandidate{cost, dist, std::move(total)});
      }
    }
    if (candidates.empty()) {
      break;
    }
    auto best = candidates.begin();
    bf::ShortestPath next;
    next.vertices = best->vertices;
    next.cost = best->cost;
    next.distance_nm = best->distance;
    next.found = true;
    candidates.erase(best);
    result.push_back(std::move(next));
  }
  return result;
}

// Per-vertex seed lookup (min seed, -1 if not an endpoint), like production.
std::vector<double> RefSeedTable(const std::vector<bf::SeededEndpoint>& eps, int n) {
  std::vector<double> seed(n, -1.0);
  for (const bf::SeededEndpoint& e : eps) {
    if (e.vertex < 0 || e.vertex >= n) {
      continue;
    }
    if (seed[e.vertex] < 0.0 || e.cost < seed[e.vertex]) {
      seed[e.vertex] = e.cost;
    }
  }
  return seed;
}

bool RefCostOfPathMulti(const bf::NavGraph& g, const std::vector<int>& path,
                        const std::vector<double>& src_seed, const std::vector<double>& goal_seed,
                        double& cost, double& dist) {
  if (path.empty()) {
    return false;
  }
  const double s = src_seed[path.front()];
  const double gg = goal_seed[path.back()];
  if (s < 0.0 || gg < 0.0) {
    return false;
  }
  double ec = 0.0, ed = 0.0;
  if (!RefCostOfPath(g, path, ec, ed)) {
    return false;
  }
  cost = s + ec + gg;
  dist = s + ed + gg;
  return true;
}

// Naive multi-source/multi-goal Yen (no Lawler): spur from index -1 every round.
std::vector<bf::ShortestPath> NaiveKShortestMulti(const bf::NavGraph& graph,
                                                  const std::vector<bf::SeededEndpoint>& sources,
                                                  const std::vector<bf::SeededEndpoint>& goals,
                                                  int k) {
  std::vector<bf::ShortestPath> result;
  if (k <= 0 || sources.empty() || goals.empty()) {
    return result;
  }
  const int n = graph.VertexCount();
  const std::vector<double> src_seed = RefSeedTable(sources, n);
  const std::vector<double> goal_seed = RefSeedTable(goals, n);
  bf::ShortestPath first = bf::FindShortestPathMulti(graph, sources, goals, bf::SearchOptions{});
  if (!first.found) {
    return result;
  }
  result.push_back(std::move(first));
  std::set<RefCandidate> candidates;
  auto add = [&](const std::vector<int>& root, const bf::ShortestPath& tail) {
    if (!tail.found || tail.vertices.empty()) {
      return;
    }
    std::vector<int> total(root.begin(), root.empty() ? root.end() : root.end() - 1);
    total.insert(total.end(), tail.vertices.begin(), tail.vertices.end());
    double cost = 0.0, dist = 0.0;
    if (RefCostOfPathMulti(graph, total, src_seed, goal_seed, cost, dist)) {
      candidates.insert(RefCandidate{cost, dist, std::move(total)});
    }
  };
  for (int kth = 1; kth < k; ++kth) {
    const std::vector<int> prev = result.back().vertices;
    for (int i = -1; i + 1 < static_cast<int>(prev.size()); ++i) {
      if (i < 0) {
        std::set<int> banned_sources;
        for (const bf::ShortestPath& p : result) {
          if (!p.vertices.empty()) {
            banned_sources.insert(p.vertices.front());
          }
        }
        std::vector<bf::SeededEndpoint> spur_sources;
        for (const bf::SeededEndpoint& s : sources) {
          if (banned_sources.count(s.vertex) == 0) {
            spur_sources.push_back(s);
          }
        }
        if (spur_sources.empty()) {
          continue;
        }
        add({}, bf::FindShortestPathMulti(graph, spur_sources, goals, bf::SearchOptions{}));
        continue;
      }
      const std::vector<int> root(prev.begin(), prev.begin() + i + 1);
      std::set<std::pair<int, int>> banned_edges;
      for (const bf::ShortestPath& p : result) {
        if (p.vertices.size() > static_cast<size_t>(i) + 1 &&
            std::equal(root.begin(), root.end(), p.vertices.begin())) {
          banned_edges.emplace(p.vertices[i], p.vertices[i + 1]);
        }
      }
      const std::set<int> banned_nodes(root.begin(), root.end() - 1);
      bf::SearchOptions opts;
      opts.node_blocked = [banned_nodes](int v) { return banned_nodes.count(v) != 0; };
      opts.edge_blocked = [banned_edges](int f, int t) { return banned_edges.count({f, t}) != 0; };
      add(root, bf::FindShortestPathMulti(graph, {bf::SeededEndpoint{prev[i], 0.0}}, goals, opts));
    }
    if (candidates.empty()) {
      break;
    }
    auto best = candidates.begin();
    bf::ShortestPath next;
    next.vertices = best->vertices;
    next.cost = best->cost;
    next.distance_nm = best->distance;
    next.found = true;
    candidates.erase(best);
    result.push_back(std::move(next));
  }
  return result;
}

// Build a random small graph: `n` waypoints scattered in a lat/lon box, then
// random directed airway segments. Deterministic given the seed.
bf::NavData MakeRandomData(std::mt19937& rng, int n, int extra_edges) {
  bf::NavData d;
  std::uniform_real_distribution<double> lat(-2.0, 2.0);
  std::uniform_real_distribution<double> lon(0.0, 6.0);
  for (int i = 0; i < n; ++i) {
    d.waypoints.push_back(bf::Waypoint{bf::Ident("W" + std::to_string(i), "ZZ"),
                                       bf::Coordinate{lat(rng), lon(rng)}, bf::WaypointKind::kFix});
  }
  // A backbone chain guarantees some connectivity, then random extra edges add
  // alternatives (which is what gives Yen multiple candidates to order).
  auto seg = [](int e) {
    bf::AirwaySegment s;
    s.name = "R" + std::to_string(e);
    s.direction = bf::AirwayDirection::kBoth;
    return s;
  };
  for (int i = 0; i + 1 < n; ++i) {
    d.airways.push_back({bf::Ident("W" + std::to_string(i), "ZZ"),
                         bf::Ident("W" + std::to_string(i + 1), "ZZ"), seg(i)});
  }
  std::uniform_int_distribution<int> pick(0, n - 1);
  for (int e = 0; e < extra_edges; ++e) {
    const int a = pick(rng);
    const int b = pick(rng);
    if (a != b) {
      d.airways.push_back({bf::Ident("W" + std::to_string(a), "ZZ"),
                           bf::Ident("W" + std::to_string(b), "ZZ"), seg(n + e)});
    }
  }
  return d;
}

TEST_CASE("Lawler matches naive Yen on hundreds of random graphs (single-source)", "[yen]") {
  std::mt19937 rng(0xB4A0);  // fixed seed: reproducible
  std::uniform_int_distribution<int> n_dist(3, 15);
  int checked = 0;
  for (int trial = 0; trial < 200; ++trial) {
    const int n = n_dist(rng);
    bf::GraphBuilder builder(MakeRandomData(rng, n, n));
    std::uniform_int_distribution<int> vpick(0, n - 1);
    const int start = vpick(rng);
    const int goal = vpick(rng);
    if (start == goal) {
      continue;
    }
    const int k = 1 + (trial % 10);  // k = 1..10
    const std::vector<bf::ShortestPath> opt =
        bf::FindKShortestPaths(builder.graph(), start, goal, k, bf::SearchOptions{});
    const std::vector<bf::ShortestPath> ref = NaiveKShortest(builder.graph(), start, goal, k);
    REQUIRE(opt.size() == ref.size());
    for (size_t i = 0; i < opt.size(); ++i) {
      CHECK(opt[i].vertices == ref[i].vertices);
      CHECK(opt[i].cost == ref[i].cost);
    }
    ++checked;
  }
  CHECK(checked > 150);  // most trials had distinct endpoints
}

TEST_CASE("Lawler matches naive Yen on hundreds of random graphs (multi-source)", "[yen]") {
  std::mt19937 rng(0x5EED);  // fixed seed: reproducible
  std::uniform_int_distribution<int> n_dist(4, 15);
  int checked = 0;
  for (int trial = 0; trial < 200; ++trial) {
    const int n = n_dist(rng);
    bf::GraphBuilder builder(MakeRandomData(rng, n, n));
    std::uniform_int_distribution<int> vpick(0, n - 1);
    std::uniform_real_distribution<double> seed(0.0, 20.0);
    // Two random sources and two random goals with random seeds.
    std::set<int> used;
    std::vector<bf::SeededEndpoint> sources, goals;
    for (int t = 0; t < 2; ++t) {
      const int v = vpick(rng);
      if (used.insert(v).second) {
        sources.push_back({v, seed(rng)});
      }
    }
    for (int t = 0; t < 2; ++t) {
      const int v = vpick(rng);
      if (used.insert(v).second) {
        goals.push_back({v, seed(rng)});
      }
    }
    if (sources.empty() || goals.empty()) {
      continue;
    }
    const int k = 1 + (trial % 10);
    const std::vector<bf::ShortestPath> opt =
        bf::FindKShortestPathsMulti(builder.graph(), sources, goals, k, bf::SearchOptions{});
    const std::vector<bf::ShortestPath> ref =
        NaiveKShortestMulti(builder.graph(), sources, goals, k);
    REQUIRE(opt.size() == ref.size());
    for (size_t i = 0; i < opt.size(); ++i) {
      CHECK(opt[i].vertices == ref[i].vertices);
      CHECK(opt[i].cost == ref[i].cost);
    }
    ++checked;
  }
  CHECK(checked > 150);
}

}  // namespace
