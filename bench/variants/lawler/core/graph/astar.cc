// SPDX-License-Identifier: MIT
#include "core/graph/astar.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace bf {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// An entry in the open set's priority queue, ordered by f = g + h (ascending).
struct QueueNode {
  double f = 0.0;
  int vertex = -1;
  bool operator>(const QueueNode& other) const { return f > other.f; }
};

// Evaluate all constraints for an edge. Returns false if any blocks it;
// otherwise accumulates soft penalties into `extra_cost`.
bool EdgeAllowed(const SearchOptions& options, const GraphEdge& edge, const Coordinate& to_coord,
                 double& extra_cost) {
  extra_cost = 0.0;
  if (options.constraints.empty() || options.request == nullptr) {
    return true;
  }
  const EdgeContext ctx{edge, to_coord};
  for (const Constraint* c : options.constraints) {
    const EdgeVerdict v = c->Evaluate(ctx, *options.request);
    if (!v.allowed) {
      return false;
    }
    extra_cost += v.extra_cost;
  }
  return true;
}

}  // namespace

ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal,
                              const SearchOptions& options) {
  ShortestPath result;
  const int n = graph.VertexCount();
  if (start < 0 || goal < 0 || start >= n || goal >= n) {
    return result;
  }
  if (options.node_filter.Blocks(start) || options.node_filter.Blocks(goal)) {
    return result;
  }

  const Coordinate goal_coord = graph.CoordOf(goal);
  // Heuristic: straight-line great-circle distance to the goal.
  auto heuristic = [&](int v) { return graph.CoordOf(v).DistanceTo(goal_coord); };

  std::vector<double> g(n, kInfinity);  // best known (effective) cost
  std::vector<double> geo(n, 0.0);      // geographic distance along best path
  std::vector<int> prev(n, -1);
  std::vector<bool> closed(n, false);

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<>> open;
  g[start] = 0.0;
  open.push(QueueNode{heuristic(start), start});

  while (!open.empty()) {
    const int u = open.top().vertex;
    open.pop();
    if (closed[u]) {
      continue;  // stale queue entry
    }
    if (u == goal) {
      break;
    }
    closed[u] = true;

    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      const int v = e->to;
      if (closed[v]) {
        continue;
      }
      if (options.node_filter.Blocks(v)) {
        continue;
      }
      if (options.edge_filter.Blocks(u, v)) {
        continue;
      }
      double extra_cost = 0.0;
      if (!EdgeAllowed(options, *e, graph.CoordOf(v), extra_cost)) {
        continue;
      }
      const double tentative = g[u] + e->distance_nm + extra_cost;
      if (tentative < g[v]) {
        g[v] = tentative;
        geo[v] = geo[u] + e->distance_nm;
        prev[v] = u;
        open.push(QueueNode{tentative + heuristic(v), v});
      }
    }
  }

  if (g[goal] == kInfinity) {
    return result;  // unreachable
  }

  // Reconstruct the path from goal back to start.
  for (int at = goal; at != -1; at = prev[at]) {
    result.vertices.push_back(at);
  }
  std::reverse(result.vertices.begin(), result.vertices.end());
  result.distance_nm = geo[goal];
  result.cost = g[goal];
  result.found = true;
  return result;
}

ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal) {
  return FindShortestPath(graph, start, goal, SearchOptions{});
}

MultiGoalHeuristic::MultiGoalHeuristic(const NavGraph& graph,
                                       const std::vector<SeededEndpoint>& goals)
    : graph_(graph), goals_(goals), cache_(graph.VertexCount(), -1.0) {}

double MultiGoalHeuristic::operator()(int vertex) const {
  double& slot = cache_[vertex];
  if (slot >= 0.0) {
    return slot;
  }
  const Coordinate c = graph_.CoordOf(vertex);
  double best = kInfinity;
  const int n = static_cast<int>(cache_.size());
  for (const SeededEndpoint& gp : goals_) {
    if (gp.vertex < 0 || gp.vertex >= n) {
      continue;
    }
    const double h = c.DistanceTo(graph_.CoordOf(gp.vertex)) + gp.cost;
    if (h < best) {
      best = h;
    }
  }
  slot = best;
  return best;
}

namespace {

// Core multi-source/multi-goal A*, parameterized on the heuristic so callers can
// supply a shared memoized one (Yen) or a throwaway inline one (single search).
template <class Heuristic>
ShortestPath RunMultiSearch(const NavGraph& graph, const std::vector<SeededEndpoint>& sources,
                            const std::vector<SeededEndpoint>& goals, const SearchOptions& options,
                            Heuristic&& heuristic) {
  ShortestPath result;
  const int n = graph.VertexCount();
  if (sources.empty() || goals.empty()) {
    return result;
  }

  // Per-vertex goal seed cost: -1 means "not a goal". A vertex may appear once
  // as a goal (connection fixes are distinct); the smallest seed wins if not.
  std::vector<double> goal_seed(n, -1.0);
  for (const SeededEndpoint& gp : goals) {
    if (gp.vertex < 0 || gp.vertex >= n) {
      continue;
    }
    if (goal_seed[gp.vertex] < 0.0 || gp.cost < goal_seed[gp.vertex]) {
      goal_seed[gp.vertex] = gp.cost;
    }
  }

  std::vector<double> g(n, kInfinity);  // best known effective cost from a source
  std::vector<double> geo(n, 0.0);      // geographic distance along best path
  std::vector<int> prev(n, -1);
  std::vector<bool> closed(n, false);

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<>> open;
  for (const SeededEndpoint& s : sources) {
    if (s.vertex < 0 || s.vertex >= n) {
      continue;
    }
    if (options.node_filter.Blocks(s.vertex)) {
      continue;
    }
    // A source's seed cost is the procedure distance already flown to reach it;
    // it counts as both effective cost and geographic distance.
    if (s.cost < g[s.vertex]) {
      g[s.vertex] = s.cost;
      geo[s.vertex] = s.cost;
      open.push(QueueNode{s.cost + heuristic(s.vertex), s.vertex});
    }
  }

  double best_total = kInfinity;  // best (g + goal seed) reached so far
  int best_goal = -1;

  while (!open.empty()) {
    const QueueNode top = open.top();
    open.pop();
    const int u = top.vertex;
    if (closed[u]) {
      continue;
    }
    // With a consistent heuristic, once the cheapest open f-value cannot beat
    // the best finished total, no remaining goal can improve it.
    if (top.f >= best_total) {
      break;
    }
    closed[u] = true;

    // Finishing at u (if it is a goal) costs g[u] plus its seed.
    if (goal_seed[u] >= 0.0) {
      const double total = g[u] + goal_seed[u];
      if (total < best_total) {
        best_total = total;
        best_goal = u;
      }
    }

    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      const int v = e->to;
      if (closed[v]) {
        continue;
      }
      if (options.node_filter.Blocks(v)) {
        continue;
      }
      if (options.edge_filter.Blocks(u, v)) {
        continue;
      }
      double extra_cost = 0.0;
      if (!EdgeAllowed(options, *e, graph.CoordOf(v), extra_cost)) {
        continue;
      }
      const double tentative = g[u] + e->distance_nm + extra_cost;
      if (tentative < g[v]) {
        g[v] = tentative;
        geo[v] = geo[u] + e->distance_nm;
        prev[v] = u;
        open.push(QueueNode{tentative + heuristic(v), v});
      }
    }
  }

  if (best_goal < 0) {
    return result;  // no source reached any goal
  }

  for (int at = best_goal; at != -1; at = prev[at]) {
    result.vertices.push_back(at);
  }
  std::reverse(result.vertices.begin(), result.vertices.end());
  // Include the chosen goal's seed cost in the reported totals.
  result.distance_nm = geo[best_goal] + goal_seed[best_goal];
  result.cost = best_total;
  result.found = true;
  return result;
}

}  // namespace

ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<SeededEndpoint>& goals,
                                   const SearchOptions& options) {
  const MultiGoalHeuristic heuristic(graph, goals);
  return RunMultiSearch(graph, sources, goals, options, heuristic);
}

ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<SeededEndpoint>& goals,
                                   const SearchOptions& options,
                                   const MultiGoalHeuristic& heuristic) {
  return RunMultiSearch(graph, sources, goals, options, heuristic);
}

const GraphEdge* SelectEdge(const NavGraph& graph, int from, int to, const SearchOptions& options,
                            double* out_cost) {
  const GraphEdge* best = nullptr;
  double best_cost = kInfinity;
  for (const GraphEdge* e = graph.EdgesBegin(from); e != graph.EdgesEnd(from); ++e) {
    if (e->to != to) {
      continue;
    }
    double extra_cost = 0.0;
    if (!EdgeAllowed(options, *e, graph.CoordOf(to), extra_cost)) {
      continue;
    }
    const double total = e->distance_nm + extra_cost;
    if (total < best_cost) {
      best_cost = total;
      best = e;
    }
  }
  if (out_cost != nullptr && best != nullptr) {
    *out_cost = best_cost;
  }
  return best;
}

}  // namespace bf
