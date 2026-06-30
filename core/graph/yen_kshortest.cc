#include "core/graph/yen_kshortest.h"

#include <algorithm>
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace bf {

namespace {

// Compute the effective cost (geographic distance + soft penalties) of a fully
// specified path, and its geographic distance. Returns false if any step is not
// a real edge or is blocked by a constraint, in which case the path is invalid.
bool CostOfPath(const NavGraph& graph, const std::vector<int>& path, const SearchOptions& options,
                double& cost, double& distance) {
  cost = 0.0;
  distance = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const int u = path[i];
    const int v = path[i + 1];
    const GraphEdge* found = nullptr;
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->to == v) {
        found = e;
        break;
      }
    }
    if (found == nullptr) {
      return false;
    }
    double extra = 0.0;
    if (!options.constraints.empty() && options.request != nullptr) {
      const EdgeContext ctx{*found, graph.CoordOf(v)};
      for (const Constraint* c : options.constraints) {
        const EdgeVerdict verdict = c->Evaluate(ctx, *options.request);
        if (!verdict.allowed) {
          return false;
        }
        extra += verdict.extra_cost;
      }
    }
    cost += found->distance_nm + extra;
    distance += found->distance_nm;
  }
  return true;
}

// A candidate path ordered by effective cost for the B set.
struct Candidate {
  double cost;
  double distance;
  std::vector<int> vertices;
  bool operator<(const Candidate& other) const {
    if (cost != other.cost) {
      return cost < other.cost;
    }
    return vertices < other.vertices;  // stable tie-break, dedupes duplicates
  }
};

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

  // Candidate set B, kept sorted/deduped by (cost, vertices).
  std::set<Candidate> candidates;

  for (int kth = 1; kth < k; ++kth) {
    const std::vector<int> prev_path = result.back().vertices;

    // Each node of the previous path (except the goal) is a spur node.
    for (size_t i = 0; i + 1 < prev_path.size(); ++i) {
      const int spur_node = prev_path[i];
      // Root = prev_path[0..i]; the spur search starts at spur_node.
      const std::vector<int> root(prev_path.begin(), prev_path.begin() + i + 1);

      // Ban the (i -> i+1) edge of every accepted/known path that shares this
      // root, so the spur search must diverge here.
      std::set<std::pair<int, int>> banned_edges;
      for (const ShortestPath& p : result) {
        if (p.vertices.size() > i + 1 && std::equal(root.begin(), root.end(), p.vertices.begin())) {
          banned_edges.emplace(p.vertices[i], p.vertices[i + 1]);
        }
      }
      // Root nodes (except the spur node) are off-limits to keep paths loopless.
      const std::set<int> banned_nodes(root.begin(), root.end() - 1);

      SearchOptions spur_opts = base_options;
      // Compose Yen's bans with any caller-supplied node/edge filter (e.g. the
      // "no transit through airports" rule) rather than overwriting it. Capture
      // the ban sets BY VALUE so the std::functions stored in spur_opts are
      // self-contained: they hold no references to these loop-local sets and are
      // therefore safe to copy, move, or hold across threads.
      auto base_node_blocked = base_options.node_blocked;
      auto base_edge_blocked = base_options.edge_blocked;
      spur_opts.node_blocked = [banned_nodes, base_node_blocked](int v) {
        return banned_nodes.count(v) != 0 || (base_node_blocked && base_node_blocked(v));
      };
      spur_opts.edge_blocked = [banned_edges, base_edge_blocked](int from, int to) {
        return banned_edges.count({from, to}) != 0 ||
               (base_edge_blocked && base_edge_blocked(from, to));
      };

      const ShortestPath spur = FindShortestPath(graph, spur_node, goal, spur_opts);
      if (!spur.found) {
        continue;
      }

      // Total path = root (without the spur node) + spur path.
      std::vector<int> total(root.begin(), root.end() - 1);
      total.insert(total.end(), spur.vertices.begin(), spur.vertices.end());

      double cost = 0.0;
      double distance = 0.0;
      if (CostOfPath(graph, total, base_options, cost, distance)) {
        candidates.insert(Candidate{cost, distance, std::move(total)});
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
    candidates.erase(best);
    result.push_back(std::move(next));
  }

  return result;
}

}  // namespace bf
