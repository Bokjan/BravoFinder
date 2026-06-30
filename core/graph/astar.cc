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
  if (options.node_blocked && (options.node_blocked(start) || options.node_blocked(goal))) {
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
      if (options.node_blocked && options.node_blocked(v)) {
        continue;
      }
      if (options.edge_blocked && options.edge_blocked(u, v)) {
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

}  // namespace bf
