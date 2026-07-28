// SPDX-License-Identifier: LGPL-3.0-or-later
#include "core/graph/astar.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace bf {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

constexpr double kPi = 3.14159265358979323846;

// Project a Coordinate onto the unit sphere as a Cartesian (x, y, z). Used by
// MultiGoalHeuristic for chord-length distances; the trig here is the only trig
// the heuristic pays per vertex (computed once on the cache miss, then reused
// across every goal).
UnitVec ProjectUnitSphere(const Coordinate& c) {
  const double lat_r = c.latitude * kPi / 180.0;
  const double lon_r = c.longitude * kPi / 180.0;
  const double cos_lat = std::cos(lat_r);
  return UnitVec{cos_lat * std::cos(lon_r), cos_lat * std::sin(lon_r), std::sin(lat_r)};
}

// Evaluate all constraints for an edge. Returns false if any blocks it;
// otherwise accumulates soft penalties into `extra_cost`.
//
// `from_coord` is the popped vertex's coordinate, hoisted out of the edge loop
// by the caller (it is constant across a vertex's out-edges). `to_coord` is
// fetched lazily -- only when constraints are actually present -- so the common
// unconstrained path (EdgeAllowed returns true on the first line) pays zero
// Coordinate fetches per edge instead of two 16-byte copies that were never
// read. `graph` is taken by reference so the lazy CoordOf(to) stays O(1).
bool EdgeAllowed(const SearchOptions& options, const GraphEdge& edge, const Coordinate& from_coord,
                 const NavGraph& graph, int to, double& extra_cost) {
  extra_cost = 0.0;
  if (options.constraints.empty() || options.request == nullptr) {
    return true;
  }
  const EdgeContext ctx{edge, from_coord, graph.CoordOf(to)};
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

const GraphEdge* SelectEdge(const NavGraph& graph, int from, int to, const SearchOptions& options,
                            double* out_cost) {
  const GraphEdge* best = nullptr;
  double best_cost = kInfinity;
  // `from` is constant across the parallel-edge loop; hoist its coordinate so
  // the unconstrained fast path (EdgeAllowed returns true immediately) fetches
  // no coordinates at all per edge.
  const Coordinate from_coord = graph.CoordOf(from);
  for (const GraphEdge* e = graph.EdgesBegin(from); e != graph.EdgesEnd(from); ++e) {
    if (e->to != to) {
      continue;
    }
    double extra_cost = 0.0;
    if (!EdgeAllowed(options, *e, from_coord, graph, to, extra_cost)) {
      continue;  // blocked on this parallel edge; try the next
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

void SearchWorkspace::Reset(int n) {
  // Grow to fit the graph; never shrink. The value arrays are not re-initialized
  // -- the generation stamp makes stale slots read as their initial value, so a
  // resize only needs to size the buffers and default the new stamp slots to a
  // value that cannot match the current generation (0, since generation_ is
  // bumped to >= 1 before any search reads a slot).
  if (static_cast<int>(stamp_.size()) < n) {
    g_.resize(n);
    geo_.resize(n);
    prev_.resize(n);
    stamp_.resize(n, 0);
    closed_stamp_.resize(n, 0);
  }
}

std::vector<double> BuildSeedTable(const std::vector<SeededEndpoint>& endpoints, int n) {
  // Per-vertex seed cost: -1 means "not an endpoint". A vertex may appear more
  // than once (distinct connection fixes); the smallest seed wins if so.
  std::vector<double> seed(n, -1.0);
  for (const SeededEndpoint& e : endpoints) {
    if (e.vertex < 0 || e.vertex >= n) {
      continue;
    }
    if (seed[e.vertex] < 0.0 || e.cost < seed[e.vertex]) {
      seed[e.vertex] = e.cost;
    }
  }
  return seed;
}

ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal,
                              const SearchOptions& options, SearchWorkspace& ws) {
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

  ws.Reset(n);
  ws.NextGeneration();

  // Reuse the workspace's open-set vector across spur searches (clear() keeps
  // capacity); drive it as a min-heap on f with push_heap/pop_heap, exactly as
  // std::priority_queue does internally.
  auto& open = ws.heap();
  open.clear();
  ws.Relax(start, 0.0, 0.0, -1);
  open.push_back(QueueNode{heuristic(start), start});
  std::push_heap(open.begin(), open.end(), std::greater<>());

  while (!open.empty()) {
    std::pop_heap(open.begin(), open.end(), std::greater<>());
    const int u = open.back().vertex;
    open.pop_back();
    if (ws.Closed(u)) {
      continue;  // stale queue entry
    }
    if (u == goal) {
      break;
    }
    ws.MarkClosed(u);

    // u is constant across its out-edges; hoist its coordinate once so the
    // unconstrained fast path fetches no per-edge coordinates.
    const Coordinate u_coord = graph.CoordOf(u);
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      const int v = e->to;
      if (ws.Closed(v)) {
        continue;
      }
      if (options.node_filter.Blocks(v)) {
        continue;
      }
      if (options.edge_filter.Blocks(u, v)) {
        continue;
      }
      double extra_cost = 0.0;
      if (!EdgeAllowed(options, *e, u_coord, graph, v, extra_cost)) {
        continue;
      }
      const double tentative = ws.G(u) + e->distance_nm + extra_cost;
      if (tentative < ws.G(v)) {
        ws.Relax(v, tentative, ws.Geo(u) + e->distance_nm, u);
        open.push_back(QueueNode{tentative + heuristic(v), v});
        std::push_heap(open.begin(), open.end(), std::greater<>());
      }
    }
  }

  if (ws.G(goal) == kInfinity) {
    return result;  // unreachable
  }

  // Reconstruct the path from goal back to start.
  for (int at = goal; at != -1; at = ws.Prev(at)) {
    result.vertices.push_back(at);
  }
  std::reverse(result.vertices.begin(), result.vertices.end());
  result.distance_nm = ws.Geo(goal);
  result.cost = ws.G(goal);
  result.found = true;
  return result;
}

ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal,
                              const SearchOptions& options) {
  SearchWorkspace ws;
  return FindShortestPath(graph, start, goal, options, ws);
}

ShortestPath FindShortestPath(const NavGraph& graph, int start, int goal) {
  return FindShortestPath(graph, start, goal, SearchOptions{});
}

MultiGoalHeuristic::MultiGoalHeuristic(const NavGraph& graph,
                                       const std::vector<SeededEndpoint>& goals)
    : graph_(graph), goals_(goals), cache_(graph.VertexCount(), -1.0) {
  // Precompute each goal's unit vector once, indexed in lock-step with goals_
  // so the hot loop subtracts pre-projected vectors instead of recomputing trig.
  const int n = graph.VertexCount();
  goals_xyz_.reserve(goals_.size());
  for (const SeededEndpoint& g : goals_) {
    goals_xyz_.push_back(
        (g.vertex >= 0 && g.vertex < n) ? ProjectUnitSphere(graph_.CoordOf(g.vertex)) : UnitVec{});
  }
}

double MultiGoalHeuristic::operator()(int vertex) const {
  double& slot = cache_[vertex];
  if (slot >= 0.0) {
    return slot;
  }
  const UnitVec v = ProjectUnitSphere(graph_.CoordOf(vertex));
  double best = kInfinity;
  const int n = static_cast<int>(cache_.size());
  for (size_t i = 0; i < goals_.size(); ++i) {
    const SeededEndpoint& gp = goals_[i];
    if (gp.vertex < 0 || gp.vertex >= n) {
      continue;
    }
    const UnitVec& g = goals_xyz_[i];
    const double dx = v.x - g.x;
    const double dy = v.y - g.y;
    const double dz = v.z - g.z;
    // Chord length on the unit sphere; <= the great-circle arc, so multiplying
    // by the earth radius stays an admissible lower bound on the NM distance.
    const double chord = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double h = kEarthRadiusNm * chord + gp.cost;
    if (h < best) {
      best = h;
    }
  }
  slot = best;
  return best;
}

// Core multi-source/multi-goal A*, reusing a caller-owned workspace so Yen's many
// spur searches share one set of per-vertex arrays instead of reallocating and
// O(V)-initializing them each time. `goal_seed[v]` is that goal's seed cost, or
// < 0 when v is not a goal (see BuildSeedTable). The workspace is reset to a
// fresh generation on entry, so callers may pass a dirty one.
ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<double>& goal_seed,
                                   const SearchOptions& options,
                                   const MultiGoalHeuristic& heuristic, SearchWorkspace& ws) {
  ShortestPath result;
  const int n = graph.VertexCount();
  if (sources.empty()) {
    return result;
  }

  ws.Reset(n);
  ws.NextGeneration();

  // Reuse the workspace's open-set vector across spur searches (clear() keeps
  // capacity); drive it as a min-heap on f with push_heap/pop_heap, exactly as
  // std::priority_queue does internally.
  auto& open = ws.heap();
  open.clear();
  for (const SeededEndpoint& s : sources) {
    if (s.vertex < 0 || s.vertex >= n) {
      continue;
    }
    if (options.node_filter.Blocks(s.vertex)) {
      continue;
    }
    // A source's seed cost is the procedure distance already flown to reach it;
    // it counts as both effective cost and geographic distance.
    if (s.cost < ws.G(s.vertex)) {
      ws.Relax(s.vertex, s.cost, s.cost, -1);
      open.push_back(QueueNode{s.cost + heuristic(s.vertex), s.vertex});
      std::push_heap(open.begin(), open.end(), std::greater<>());
    }
  }

  double best_total = kInfinity;  // best (g + goal seed) reached so far
  int best_goal = -1;

  while (!open.empty()) {
    std::pop_heap(open.begin(), open.end(), std::greater<>());
    const QueueNode top = open.back();
    open.pop_back();
    const int u = top.vertex;
    if (ws.Closed(u)) {
      continue;
    }
    // With a consistent heuristic, once the cheapest open f-value cannot beat
    // the best finished total, no remaining goal can improve it.
    if (top.f >= best_total) {
      break;
    }
    ws.MarkClosed(u);

    // Finishing at u (if it is a goal) costs g[u] plus its seed.
    if (goal_seed[u] >= 0.0) {
      const double total = ws.G(u) + goal_seed[u];
      if (total < best_total) {
        best_total = total;
        best_goal = u;
      }
    }

    // u is constant across its out-edges; hoist its coordinate once so the
    // unconstrained fast path fetches no per-edge coordinates.
    const Coordinate u_coord = graph.CoordOf(u);
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      const int v = e->to;
      if (ws.Closed(v)) {
        continue;
      }
      if (options.node_filter.Blocks(v)) {
        continue;
      }
      if (options.edge_filter.Blocks(u, v)) {
        continue;
      }
      double extra_cost = 0.0;
      if (!EdgeAllowed(options, *e, u_coord, graph, v, extra_cost)) {
        continue;
      }
      const double tentative = ws.G(u) + e->distance_nm + extra_cost;
      if (tentative < ws.G(v)) {
        ws.Relax(v, tentative, ws.Geo(u) + e->distance_nm, u);
        open.push_back(QueueNode{tentative + heuristic(v), v});
        std::push_heap(open.begin(), open.end(), std::greater<>());
      }
    }
  }

  if (best_goal < 0) {
    return result;  // no source reached any goal
  }

  for (int at = best_goal; at != -1; at = ws.Prev(at)) {
    result.vertices.push_back(at);
  }
  std::reverse(result.vertices.begin(), result.vertices.end());
  // Include the chosen goal's seed cost in the reported totals.
  result.distance_nm = ws.Geo(best_goal) + goal_seed[best_goal];
  result.cost = best_total;
  result.found = true;
  return result;
}

ShortestPath FindShortestPathMulti(const NavGraph& graph,
                                   const std::vector<SeededEndpoint>& sources,
                                   const std::vector<SeededEndpoint>& goals,
                                   const SearchOptions& options) {
  const MultiGoalHeuristic heuristic(graph, goals);
  const std::vector<double> goal_seed = BuildSeedTable(goals, graph.VertexCount());
  SearchWorkspace ws;
  return FindShortestPathMulti(graph, sources, goal_seed, options, heuristic, ws);
}

}  // namespace bf
