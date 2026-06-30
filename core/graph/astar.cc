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

}  // namespace

ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal) {
  ShortestPath result;
  const int n = graph.VertexCount();
  if (start < 0 || goal < 0 || start >= n || goal >= n) {
    return result;
  }

  const Coordinate goal_coord = graph.CoordOf(goal);
  // Heuristic: straight-line great-circle distance to the goal.
  auto heuristic = [&](int v) { return graph.CoordOf(v).DistanceTo(goal_coord); };

  std::vector<double> g(n, kInfinity);  // best known cost from start
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
      const double tentative = g[u] + e->distance_nm;
      if (tentative < g[v]) {
        g[v] = tentative;
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
  result.distance_nm = g[goal];
  result.found = true;
  return result;
}

}  // namespace bf
