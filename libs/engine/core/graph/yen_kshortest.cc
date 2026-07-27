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
bool CostOfPath(const NavGraph& graph, const std::vector<int>& path, const SearchOptions& options,
                double& cost, double& distance) {
  cost = 0.0;
  distance = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
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
                     const SearchOptions& options, double& cost, double& distance) {
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
  if (!CostOfPath(graph, path, options, enroute_cost, enroute_dist)) {
    return false;
  }
  cost = s + enroute_cost + g;
  distance = s + enroute_dist + g;
  return true;
}

}  // namespace

std::vector<ShortestPath> FindKShortestPaths(const NavGraph& graph, int start, int goal, int k,
                                             const SearchOptions& base_options) {
  std::vector<ShortestPath> result;
  if (k <= 0) {
    return result;
  }

  ShortestPath first = FindShortestPath(graph, start, goal, base_options);
  if (!first.found) {
    return result;
  }
  result.push_back(std::move(first));

  // One workspace reused by every spur search: its per-vertex arrays are
  // allocated once and cleared in O(1) between searches via a generation stamp,
  // instead of each spur reallocating and O(V)-initializing fresh arrays (the
  // same optimization FindKShortestPathsMulti already applies). Stack-local to
  // this call, so concurrent queries never share it.
  SearchWorkspace ws;

  // Candidate set B, kept sorted/deduped by (cost, vertices). B persists across
  // the outer k iterations (Lawler): candidates not chosen this round stay for
  // the next.
  std::set<Candidate> candidates;

  // Lawler's optimization: the deviation index of the most recently accepted
  // path -- the spur position at which it branched from its parent. Spur nodes
  // before this index would reproduce root prefixes already fully explored in an
  // earlier round, so the next round starts spurring here. The first (shortest)
  // path has no parent; index 0 spurs it in full.
  int last_deviation = 0;

  for (int kth = 1; kth < k; ++kth) {
    const std::vector<int>& prev_path = result.back().vertices;

    // Spur from the previous path's deviation index onward (Lawler), not from 0.
    for (size_t i = static_cast<size_t>(last_deviation); i + 1 < prev_path.size(); ++i) {
      const int spur_node = prev_path[i];
      // Root = prev_path[0..i]; the spur search starts at spur_node.
      const std::vector<int> root(prev_path.begin(), prev_path.begin() + i + 1);

      // Ban the (i -> i+1) edge of every accepted/known path that shares this
      // root, so the spur search must diverge here. Built as a const sorted
      // vector (the IIFE sorts in place, then binds to const) so binary_search
      // is valid by construction -- the type system prevents any later write.
      const std::vector<int64_t> banned_edges = [&] {
        std::vector<int64_t> v;
        v.reserve(result.size());
        for (const ShortestPath& p : result) {
          if (p.vertices.size() > i + 1 &&
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
      // through airports" rule) rather than overwriting it: spur_opts starts as
      // a copy of base_options (so the base node_filter's airport range is
      // inherited), then this spur's banned nodes/edges are layered on as sorted
      // vectors the filters binary_search. The const vectors outlive the spur
      // search -- same loop iteration, stack-local.
      spur_opts.node_filter.banned = &banned_nodes;
      spur_opts.edge_filter.banned = &banned_edges;

      const ShortestPath spur = FindShortestPath(graph, spur_node, goal, spur_opts, ws);
      if (!spur.found) {
        continue;
      }

      // Total path = root (without the spur node) + spur path.
      std::vector<int> total(root.begin(), root.end() - 1);
      total.insert(total.end(), spur.vertices.begin(), spur.vertices.end());

      double cost = 0.0;
      double distance = 0.0;
      if (CostOfPath(graph, total, base_options, cost, distance)) {
        candidates.insert(Candidate{cost, distance, std::move(total), static_cast<int>(i)});
      }
    }

    if (candidates.empty()) {
      break;
    }
    // Accept the cheapest candidate not already in the result set.
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

  // The goal set is fixed for the whole run, so h(v) is constant per vertex.
  // Build one memoized heuristic and share it across the first search and every
  // spur search, instead of each search re-sweeping all goals on every pop.
  const MultiGoalHeuristic heuristic(graph, goals);

  // One workspace reused by every spur search: its per-vertex arrays are
  // allocated once and cleared in O(1) between searches via a generation stamp,
  // rather than reallocated and O(V)-initialized on each of the hundreds of
  // spurs. Stack-local to this call, so concurrent queries never share it.
  SearchWorkspace ws;

  ShortestPath first =
      FindShortestPathMulti(graph, sources, goal_seed, base_options, heuristic, ws);
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
    if (CostOfPathMulti(graph, total, source_seed, goal_seed, base_options, cost, distance)) {
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
            FindShortestPathMulti(graph, spur_sources, goal_seed, base_options, heuristic, ws);
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
      // stitched path's first vertex.
      const ShortestPath spur = FindShortestPathMulti(graph, {SeededEndpoint{spur_node, 0.0}},
                                                      goal_seed, spur_opts, heuristic, ws);
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
