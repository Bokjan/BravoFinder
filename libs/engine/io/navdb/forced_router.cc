// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/navdb/forced_router.h"

#include <algorithm>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/domain/ident.h"
#include "core/graph/yen_kshortest.h"
#include "io/build/graph_builder.h"

namespace bf {

namespace {

// How many per-hop combinations FindPaths stitches before it settles for the
// best routes found so far. Far above what a normal request needs (a handful),
// it only bounds the case where large turns at the forced fixes keep the lower
// bound below every stitched route.
constexpr int kMaxStitchedPicks = 4096;

// Concatenate hop paths that meet at their shared seam vertices into one route
// and re-cost it end to end (CostOfPathMulti, so the turns at the seams count).
// Returns found=false if the route repeats a vertex or the re-cost rejects it.
ShortestPath StitchAndCost(const NavGraph& graph, const std::vector<const std::vector<int>*>& parts,
                           const std::vector<double>& source_seed,
                           const std::vector<double>& goal_seed,
                           const std::vector<double>& source_bearing,
                           const std::vector<double>& goal_bearing, const SearchOptions& options) {
  ShortestPath out;
  std::vector<int> path;
  for (const std::vector<int>* part : parts) {
    if (part->empty()) {
      return out;
    }
    if (path.empty()) {
      path = *part;
      continue;
    }
    if (path.back() != part->front()) {
      return out;  // seam mismatch (should not happen: seam == forced fix)
    }
    // Use explicit loop instead of range-insert to avoid a GCC 14
    // -Wstringop-overflow= false positive on __builtin_memcpy inside
    // std::vector::insert(range). Reserve upfront so the loop allocates at
    // most once, matching the original insert behaviour.
    path.reserve(path.size() + part->size() - 1);
    for (size_t i = 1; i < part->size(); ++i) {
      path.push_back((*part)[i]);
    }
  }
  std::unordered_set<int> seen;
  seen.reserve(path.size());
  for (const int v : path) {
    if (!seen.insert(v).second) {
      return out;  // cycle at a seam -> not a simple route
    }
  }
  double cost = 0.0;
  double dist = 0.0;
  if (!CostOfPathMulti(graph, path, source_seed, goal_seed, source_bearing, goal_bearing, options,
                       cost, dist)) {
    return out;
  }
  out.vertices = std::move(path);
  out.distance_nm = dist;
  out.cost = cost;
  out.found = true;
  return out;
}

}  // namespace

int ForcedRouter::ResolvePoint(const GraphBuilder& builder, const std::string& token,
                               const Coordinate& from, const Coordinate& to, std::string& echo,
                               bool& is_airport) {
  is_airport = false;
  const std::string up = ToUpper(token);
  const size_t slash = up.find('/');
  if (slash != std::string::npos) {
    const Ident id(up.substr(0, slash), up.substr(slash + 1));
    const int v = builder.VertexByIdent(id);
    if (v < 0) {
      return -1;
    }
    if (builder.IsAirport(v)) {
      is_airport = true;
      return -1;
    }
    echo = id.ident + "/" + id.arinc424_icao_code;
    return v;
  }
  // Bare ident: prefer a match a route can actually pass through -- one with
  // both an inbound and an outbound airway edge -- since a via point off the
  // network can never be routed and a closer namesake on it can. Within the
  // same tier, choose the non-airport match minimizing the added detour
  // d(from,v) + d(v,to). The constant d(from,to) is omitted since it is the
  // same for every candidate and does not change the argmin. Ties break on the
  // lowest vertex index for determinism.
  const auto tier = [&builder](int v) {
    if (builder.HasInbound(v) && builder.HasOutbound(v)) {
      return 0;
    }
    return builder.OnNetwork(v) ? 1 : 2;
  };
  int best = -1;
  int best_tier = 0;
  double best_detour = 0.0;
  bool saw_airport = false;
  for (const int v : builder.VerticesByIdent(up)) {
    if (builder.IsAirport(v)) {
      saw_airport = true;
      continue;
    }
    const Coordinate c = builder.graph().CoordOf(v);
    const double detour = from.DistanceTo(c) + c.DistanceTo(to);
    const int t = tier(v);
    if (best < 0 || t < best_tier || (t == best_tier && detour < best_detour)) {
      best = v;
      best_tier = t;
      best_detour = detour;
    }
  }
  if (best < 0) {
    is_airport = saw_airport;  // only matches were airports
    return -1;
  }
  const Ident id = builder.IdentOf(best);
  echo = id.ident + "/" + id.arinc424_icao_code;
  return best;
}

std::vector<ShortestPath> ForcedRouter::FindPaths(const NavGraph& graph,
                                                  const std::vector<SeededEndpoint>& sources,
                                                  const std::vector<SeededEndpoint>& goals,
                                                  const std::vector<int>& forced, int k,
                                                  const SearchOptions& options) {
  std::vector<ShortestPath> results;
  if (k <= 0 || forced.empty()) {
    return results;
  }

  // Build each hop's endpoint sets, then its up-to-k candidate paths.
  const size_t hops = forced.size() + 1;
  std::vector<std::vector<ShortestPath>> segments;
  segments.reserve(hops);
  for (size_t h = 0; h < hops; ++h) {
    const std::vector<SeededEndpoint> hop_sources =
        (h == 0) ? sources : std::vector<SeededEndpoint>{SeededEndpoint{forced[h - 1], 0.0}};
    const std::vector<SeededEndpoint> hop_goals =
        (h + 1 == hops) ? goals : std::vector<SeededEndpoint>{SeededEndpoint{forced[h], 0.0}};
    std::vector<ShortestPath> cands =
        FindKShortestPathsMulti(graph, hop_sources, hop_goals, k, options);
    if (cands.empty()) {
      return results;  // a hop is unroutable -> no forced route exists
    }
    // The merge below needs each list in ascending cost order. Yen accepts in
    // that order, but with the turn penalty on its first path comes from the
    // greedy single-state A*, which a later candidate can undercut, so sort.
    std::stable_sort(cands.begin(), cands.end(),
                     [](const ShortestPath& a, const ShortestPath& b) { return a.cost < b.cost; });
    segments.push_back(std::move(cands));
  }

  // Overall endpoint seed/bearing tables so a stitched path can be re-costed
  // with CostOfPathMulti: summing per-hop costs drops the turn penalty at each
  // via seam (hop endpoints carry kNoBearing), which systematically under-costs
  // sharp via handoffs.
  const int n = graph.VertexCount();
  const std::vector<double> source_seed = BuildSeedTable(sources, n);
  const std::vector<double> goal_seed = BuildSeedTable(goals, n);
  const std::vector<double> source_bearing = BuildBearingTable(sources, n);
  const std::vector<double> goal_bearing = BuildBearingTable(goals, n);

  // Stitch one combination (one candidate index per segment) into a full path
  // and re-cost it under the full turn model. Returns found=false if the
  // result has a cycle (two hops share a vertex) or re-costing rejects it.
  auto stitch = [&](const std::vector<int>& pick) -> ShortestPath {
    std::vector<const std::vector<int>*> parts;
    parts.reserve(hops);
    for (size_t h = 0; h < hops; ++h) {
      parts.push_back(&segments[h][pick[h]].vertices);
    }
    return StitchAndCost(graph, parts, source_seed, goal_seed, source_bearing, goal_bearing,
                         options);
  };

  // Best-first enumeration of the Cartesian product of segment candidates. Each
  // hop's list is in ascending cost order (Yen accepts in that order), so the
  // sum of the picked hop costs never decreases along a pick's successors, and
  // it is a lower bound on the stitched cost: re-costing adds only the turns at
  // the forced fixes, which no single hop sees. So picks come off the heap in
  // lower-bound order, every valid stitch waits in `ready`, and the cheapest
  // ready route is emitted once no unexplored pick can undercut it -- output is
  // in true cost order. A pick whose stitch is invalid (two hops share a
  // vertex) is still expanded: the all-best pick failing must not end the
  // search, since a cheap alternative in one hop usually avoids the clash.
  //
  // Each pick is generated once without a visited set: a pick advanced at hop
  // `last` only advances hops >= last, so every index vector has exactly one
  // parent chain. A budget bounds the work when seam turns keep the bound low.
  struct PickItem {
    double lower;
    std::vector<int> pick;
    size_t last;
  };
  const auto pick_after = [](const PickItem& a, const PickItem& b) { return a.lower > b.lower; };
  const auto route_after = [](const ShortestPath& a, const ShortestPath& b) {
    if (a.cost != b.cost) {
      return a.cost > b.cost;
    }
    return a.vertices > b.vertices;
  };
  std::vector<PickItem> frontier;
  std::vector<ShortestPath> ready;
  const auto lower_of = [&](const std::vector<int>& pick) {
    double sum = 0.0;
    for (size_t h = 0; h < hops; ++h) {
      sum += segments[h][pick[h]].cost;
    }
    return sum;
  };
  {
    std::vector<int> root(hops, 0);
    const double lower = lower_of(root);
    frontier.push_back(PickItem{lower, std::move(root), 0});
  }
  int explored = 0;
  while (static_cast<int>(results.size()) < k) {
    const bool can_explore = !frontier.empty() && explored < kMaxStitchedPicks;
    if (!ready.empty() && (!can_explore || ready.front().cost <= frontier.front().lower)) {
      std::pop_heap(ready.begin(), ready.end(), route_after);
      results.push_back(std::move(ready.back()));
      ready.pop_back();
      continue;
    }
    if (!can_explore) {
      break;
    }
    std::pop_heap(frontier.begin(), frontier.end(), pick_after);
    PickItem item = std::move(frontier.back());
    frontier.pop_back();
    ++explored;
    ShortestPath stitched = stitch(item.pick);
    if (stitched.found) {
      ready.push_back(std::move(stitched));
      std::push_heap(ready.begin(), ready.end(), route_after);
    }
    for (size_t h = item.last; h < hops; ++h) {
      if (item.pick[h] + 1 < static_cast<int>(segments[h].size())) {
        std::vector<int> next = item.pick;
        next[h] += 1;
        const double lower = lower_of(next);
        frontier.push_back(PickItem{lower, std::move(next), h});
        std::push_heap(frontier.begin(), frontier.end(), pick_after);
      }
    }
  }
  if (!results.empty()) {
    return results;
  }

  // Every combination of per-hop alternatives shares a vertex between two hops
  // (each hop's few cheapest paths all cross the same fix). Fall back to one
  // route built hop by hop, each hop barred from the vertices already used and
  // from the forced fixes still ahead, so the result is simple by construction.
  // Each hop starts with the heading the previous one arrived on, so the turn
  // at the forced fix is priced too.
  std::vector<int> path;
  for (size_t h = 0; h < hops; ++h) {
    std::vector<int> banned(path.begin(), path.empty() ? path.end() : path.end() - 1);
    for (size_t j = h + 1; j < forced.size(); ++j) {
      banned.push_back(forced[j]);
    }
    if (options.node_filter.banned != nullptr) {
      banned.insert(banned.end(), options.node_filter.banned->begin(),
                    options.node_filter.banned->end());
    }
    std::sort(banned.begin(), banned.end());
    banned.erase(std::unique(banned.begin(), banned.end()), banned.end());
    SearchOptions hop_options = options;
    hop_options.node_filter.banned = &banned;
    double arrival_bearing = kNoBearing;
    if (path.size() >= 2) {
      arrival_bearing = graph.CoordOf(path[path.size() - 2]).BearingTo(graph.CoordOf(path.back()));
    }
    const std::vector<SeededEndpoint> hop_sources =
        (h == 0) ? sources
                 : std::vector<SeededEndpoint>{SeededEndpoint{forced[h - 1], 0.0, arrival_bearing}};
    const std::vector<SeededEndpoint> hop_goals =
        (h + 1 == hops) ? goals : std::vector<SeededEndpoint>{SeededEndpoint{forced[h], 0.0}};
    const ShortestPath hop = FindShortestPathMulti(graph, hop_sources, hop_goals, hop_options);
    if (!hop.found || hop.vertices.empty()) {
      return results;
    }
    for (size_t i = path.empty() ? 0 : 1; i < hop.vertices.size(); ++i) {
      path.push_back(hop.vertices[i]);
    }
  }
  ShortestPath greedy =
      StitchAndCost(graph, {&path}, source_seed, goal_seed, source_bearing, goal_bearing, options);
  if (greedy.found) {
    results.push_back(std::move(greedy));
  }
  return results;
}

}  // namespace bf
