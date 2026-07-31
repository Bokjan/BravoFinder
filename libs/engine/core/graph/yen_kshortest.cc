// SPDX-License-Identifier: LGPL-3.0-or-later
#include "core/graph/yen_kshortest.h"

#include <algorithm>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

namespace bf {

namespace {

// Compute the effective cost (geographic distance + soft penalties) of a fully
// specified path, and its geographic distance. Returns false if any step has no
// usable edge to the next vertex, or is blocked by a constraint on every
// parallel edge. For each step, when several parallel edges connect u->v (the
// same airway over different flight-level bands, or two airways sharing both
// endpoints), SelectEdge picks the cheapest allowed one by (distance_nm + soft
// penalty) -- the SAME routine A* relaxation and route-leg labeling use, so this
// re-costing cannot drift from the search's own edge choice (a hand-rolled copy
// of that selection previously risked exactly that drift).
//
// `src_bearing`/`goal_bearing` are the procedure headings at the path's
// endpoints (inbound at the source fix, outbound at the goal fix), or -1 when
// unknown. When the turn-angle penalty is enabled, the per-vertex turn cost is
// added here too -- using the same headings the A* relaxation used -- so Yen's
// candidate ordering and reported cost stay on the same cost model as the search
// (gap A: a path-dependent penalty that only lived in the search loop
// would make the B-set rank paths by a different cost than they were found with).
bool CostOfPath(const NavGraph& graph, const std::vector<int>& path, const SearchOptions& options,
                double src_bearing, double goal_bearing, double& cost, double& distance) {
  cost = 0.0;
  distance = 0.0;
  const size_t m = path.size();
  if (m < 2) {
    return true;  // no edges; CostOfPathMulti handles the endpoint seeds
  }
  const TurnPenalty& turn = options.turn_penalty;
  // Per-edge outbound bearings, computed only when the turn penalty is active so
  // the re-costing hot path stays trig-free when the penalty is disabled.
  std::vector<double> edge_bearing;
  if (turn.enabled) {
    edge_bearing.resize(m - 1);
  }
  for (size_t i = 0; i + 1 < m; ++i) {
    const int u = path[i];
    const int v = path[i + 1];
    // Delegate the parallel-edge choice to SelectEdge -- the SAME routine A*
    // relaxation and route-leg labeling use -- so this re-costing can never
    // drift from the search's own edge selection. SelectEdge hands back the
    // winner's effective cost (distance + penalties) via out_cost, so no
    // constraint is re-evaluated here.
    double edge_cost = 0.0;
    const GraphEdge* e = SelectEdge(graph, u, v, options, &edge_cost);
    if (e == nullptr) {
      return false;  // no allowed edge u->v
    }
    cost += edge_cost;
    distance += e->distance_nm;
    if (turn.enabled) {
      edge_bearing[i] = graph.CoordOf(u).BearingTo(graph.CoordOf(v));
    }
  }
  if (turn.enabled) {
    // Turn at the source fix: procedure inbound heading -> first edge outbound.
    if (src_bearing >= 0.0) {
      cost += turn(TurnAngleDeg(src_bearing, edge_bearing[0]));
    }
    // Turns at interior vertices: prev edge outbound -> next edge outbound.
    for (size_t i = 1; i + 1 < m; ++i) {
      cost += turn(TurnAngleDeg(edge_bearing[i - 1], edge_bearing[i]));
    }
    // Turn at the goal fix: last edge inbound -> procedure outbound heading.
    if (goal_bearing >= 0.0) {
      cost += turn(TurnAngleDeg(edge_bearing[m - 2], goal_bearing));
    }
  }
  return true;
}

// A candidate path ordered by effective cost for the B set. `deviation` records
// the spur index at which this candidate branched from its parent accepted path
// (Lawler's optimization): once accepted, the next round only needs to spur from
// this index onward, since spurs before it merely regenerate paths already
// considered in an earlier round. It is metadata, NOT part of the ordering key.
struct Candidate {
  double cost;
  double distance;
  std::vector<int> vertices;
  int deviation = 0;
  bool operator<(const Candidate& other) const {
    if (cost != other.cost) {
      return cost < other.cost;
    }
    return vertices < other.vertices;  // stable tie-break, dedupes duplicates
  }
};

// Effective cost and geographic distance of a full source..goal path, including
// both endpoints' seed costs. Returns false if any interior step is not a real
// edge, is blocked by a constraint, or an endpoint is not actually seeded.
bool CostOfPathMulti(const NavGraph& graph, const std::vector<int>& path,
                     const std::vector<double>& source_seed, const std::vector<double>& goal_seed,
                     const std::vector<double>& source_bearing,
                     const std::vector<double>& goal_bearing, const SearchOptions& options,
                     double& cost, double& distance) {
  if (path.empty()) {
    return false;
  }
  const double s = source_seed[path.front()];
  const double g = goal_seed[path.back()];
  if (s < 0.0 || g < 0.0) {
    return false;
  }
  double enroute_cost = 0.0;
  double enroute_dist = 0.0;
  if (!CostOfPath(graph, path, options, source_bearing[path.front()], goal_bearing[path.back()],
                  enroute_cost, enroute_dist)) {
    return false;
  }
  cost = s + enroute_cost + g;
  distance = s + enroute_dist + g;
  return true;
}

}  // namespace

std::vector<ShortestPath> FindKShortestPaths(const NavGraph& graph, int start, int goal, int k,
                                             const SearchOptions& base_options) {
  // Delegate to the multi-source Yen search with a single zero-seed source/goal.
  // The single-source entry point is kept as a minimal API and unit-test seam,
  // but no longer maintains a separate Yen loop: one loop serves both, so unit
  // tests exercise the production Yen path and the single-source heuristic cannot
  // diverge from the multi-source one. The multi-source search manages its own
  // reused workspace internally, so workspace sharing across spurs is preserved.
  return FindKShortestPathsMulti(graph, {SeededEndpoint{start, 0.0}}, {SeededEndpoint{goal, 0.0}},
                                 k, base_options);
}

std::vector<ShortestPath> FindKShortestPathsMulti(const NavGraph& graph,
                                                  const std::vector<SeededEndpoint>& sources,
                                                  const std::vector<SeededEndpoint>& goals, int k,
                                                  const SearchOptions& base_options) {
  std::vector<ShortestPath> result;
  if (k <= 0 || sources.empty() || goals.empty()) {
    return result;
  }
  const int n = graph.VertexCount();
  const std::vector<double> source_seed = BuildSeedTable(sources, n);
  const std::vector<double> goal_seed = BuildSeedTable(goals, n);
  // Per-vertex procedure headings at the endpoints, for the turn-angle penalty.
  // `no_source_bearing` is an all-(-1) table for mid-path spur searches whose
  // spur node is not a real SID source (it has no procedure inbound heading).
  const std::vector<double> source_bearing = BuildBearingTable(sources, n);
  const std::vector<double> goal_bearing = BuildBearingTable(goals, n);
  const std::vector<double> no_source_bearing(n, -1.0);

  // The goal set is fixed for the whole run, so h(v) is constant per vertex.
  // Build one memoized heuristic and share it across the first search and every
  // spur search, instead of each search re-sweeping all goals on every pop.
  const MultiGoalHeuristic heuristic(graph, goals);

  // One workspace reused by every spur search: its per-vertex arrays are
  // allocated once and cleared in O(1) between searches via a generation stamp,
  // rather than reallocated and O(V)-initialized on each of the hundreds of
  // spurs. Stack-local to this call, so concurrent queries never share it.
  SearchWorkspace ws;

  ShortestPath first = FindShortestPathMulti(graph, sources, goal_seed, source_bearing,
                                             goal_bearing, base_options, heuristic, ws);
  if (!first.found) {
    return result;
  }
  result.push_back(std::move(first));

  // Candidate set B, kept sorted/deduped by (cost, vertices). B persists across
  // the outer k iterations (Lawler).
  std::set<Candidate> candidates;

  // Run one spur search and fold the resulting full path into the candidate set.
  // `root` is the prefix shared with the previous path (empty for the source-
  // level spur); `spur_tail` is the freshly searched suffix starting at the spur
  // node. The two are stitched (root minus its last node, which equals the spur
  // node, then the tail) and re-costed end to end including both seeds.
  // `deviation` is the spur index (-1 for the super-source spur) recorded on the
  // candidate for Lawler's next-round start.
  auto add_candidate = [&](const std::vector<int>& root, const ShortestPath& spur_tail,
                           int deviation) {
    if (!spur_tail.found || spur_tail.vertices.empty()) {
      return;
    }
    std::vector<int> total(root.begin(), root.empty() ? root.end() : root.end() - 1);
    total.insert(total.end(), spur_tail.vertices.begin(), spur_tail.vertices.end());
    double cost = 0.0;
    double distance = 0.0;
    if (CostOfPathMulti(graph, total, source_seed, goal_seed, source_bearing, goal_bearing,
                        base_options, cost, distance)) {
      candidates.insert(Candidate{cost, distance, std::move(total), deviation});
    }
  };

  // Lawler's optimization: deviation index of the most recently accepted path.
  // -1 means the super-source spur (a different starting connection fix); the
  // first accepted path conceptually deviates there, so the search starts at -1.
  int last_deviation = -1;

  for (int kth = 1; kth < k; ++kth) {
    const std::vector<int>& prev_path = result.back().vertices;

    // Spur nodes are every node of the previous path except the goal, plus a
    // conceptual super-source at index -1 whose "edge" to the first node selects
    // the starting connection fix. This super-source spur is the extension over
    // standard (single-source, single-goal) Yen: banning the first-node choice
    // forces the search onto a different SID/STAR entry, which is how candidates
    // that use a different source fix arise. Lawler: start at the previous path's
    // deviation index rather than always at -1.
    for (int i = last_deviation; i + 1 < static_cast<int>(prev_path.size()); ++i) {
      if (i < 0) {
        // Source-level spur: re-run the multi-source search with every starting
        // fix used by an accepted path that shares the (empty) root banned, so a
        // different source fix must be chosen.
        std::set<int> banned_sources;
        for (const ShortestPath& p : result) {
          if (!p.vertices.empty()) {
            banned_sources.insert(p.vertices.front());
          }
        }
        std::vector<SeededEndpoint> spur_sources;
        for (const SeededEndpoint& s : sources) {
          if (banned_sources.count(s.vertex) == 0) {
            spur_sources.push_back(s);
          }
        }
        if (spur_sources.empty()) {
          continue;
        }
        const ShortestPath spur =
            FindShortestPathMulti(graph, spur_sources, goal_seed, source_bearing, goal_bearing,
                                  base_options, heuristic, ws);
        add_candidate({}, spur, /*deviation=*/-1);
        continue;
      }

      const int spur_node = prev_path[i];
      const std::vector<int> root(prev_path.begin(), prev_path.begin() + i + 1);

      // Ban the (i -> i+1) edge of every accepted/known path that shares this
      // root, so the spur search must diverge here. Built as a const sorted
      // vector (the IIFE sorts in place, then binds to const) so binary_search
      // is valid by construction -- the type system prevents any later write.
      const std::vector<int64_t> banned_edges = [&] {
        std::vector<int64_t> v;
        v.reserve(result.size());
        for (const ShortestPath& p : result) {
          if (p.vertices.size() > static_cast<size_t>(i) + 1 &&
              std::equal(root.begin(), root.end(), p.vertices.begin())) {
            v.push_back(EdgeKey(p.vertices[i], p.vertices[i + 1]));
          }
        }
        std::sort(v.begin(), v.end());
        return v;
      }();
      // Root nodes (except the spur node) are off-limits to keep paths loopless.
      const std::vector<int> banned_nodes = [&] {
        std::vector<int> v(root.begin(), root.end() - 1);
        std::sort(v.begin(), v.end());
        return v;
      }();

      SearchOptions spur_opts = base_options;
      // Compose Yen's bans with any caller-supplied filter (e.g. the "no transit
      // through airports" rule): spur_opts starts as a copy of base_options (so
      // the base node_filter's airport range is inherited), then this spur's
      // banned nodes/edges are layered on as sorted vectors the filters
      // binary_search. The const vectors outlive the spur search -- same loop
      // iteration, stack-local. No std::function is stored, so there is no
      // self-contained-copy concern and no shared mutable state (concurrency
      // contract intact).
      spur_opts.node_filter.banned = &banned_nodes;
      spur_opts.edge_filter.banned = &banned_edges;

      // Single-source (the spur node) -> any goal. The spur node's own seed is
      // irrelevant here; CostOfPathMulti re-applies the true source seed from the
      // stitched path's first vertex. The spur node has no procedure inbound
      // heading, so pass the all-(-1) source bearing table -- no SID-exit turn
      // penalty is applied at the spur node during the search (the re-cost still
      // applies the real source bearing at the stitched path's true front fix).
      const ShortestPath spur =
          FindShortestPathMulti(graph, {SeededEndpoint{spur_node, 0.0}}, goal_seed,
                                no_source_bearing, goal_bearing, spur_opts, heuristic, ws);
      add_candidate(root, spur, /*deviation=*/i);
    }

    if (candidates.empty()) {
      break;
    }
    auto best = candidates.begin();
    ShortestPath next;
    next.vertices = best->vertices;
    next.cost = best->cost;
    next.distance_nm = best->distance;
    next.found = true;
    last_deviation = best->deviation;
    candidates.erase(best);
    result.push_back(std::move(next));
  }

  return result;
}

}  // namespace bf
