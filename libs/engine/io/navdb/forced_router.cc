// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/navdb/forced_router.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/domain/ident.h"
#include "core/graph/yen_kshortest.h"
#include "io/build/graph_builder.h"

namespace bf {

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
  // Bare ident: choose the non-airport match minimizing the added detour
  // d(from,v) + d(v,to). The constant d(from,to) is omitted since it is the
  // same for every candidate and does not change the argmin. Ties break on the
  // lowest vertex index for determinism.
  int best = -1;
  double best_detour = 0.0;
  bool saw_airport = false;
  for (const int v : builder.VerticesByIdent(up)) {
    if (builder.IsAirport(v)) {
      saw_airport = true;
      continue;
    }
    const Coordinate c = builder.graph().CoordOf(v);
    const double detour = from.DistanceTo(c) + c.DistanceTo(to);
    if (best < 0 || detour < best_detour) {
      best = v;
      best_detour = detour;
    }
  }
  if (best < 0) {
    is_airport = saw_airport;  // only matches were airports
    return -1;
  }
  echo = builder.IdentOf(best).ident + "/" + builder.IdentOf(best).arinc424_icao_code;
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
  // segments do not meet, the result has a cycle, or re-costing rejects it.
  auto stitch = [&](const std::vector<int>& pick) -> ShortestPath {
    ShortestPath out;
    std::vector<int> path;
    for (size_t h = 0; h < hops; ++h) {
      const ShortestPath& seg = segments[h][pick[h]];
      if (seg.vertices.empty()) {
        return out;
      }
      if (path.empty()) {
        path = seg.vertices;
      } else {
        if (path.back() != seg.vertices.front()) {
          return out;  // seam mismatch (should not happen: seam == forced fix)
        }
        if (seg.vertices.size() > 1) {
          // Use explicit loop instead of range-insert to avoid a GCC 14
          // -Wstringop-overflow= false positive on __builtin_memcpy inside
          // std::vector::insert(range). Reserve upfront so the loop
          // allocates at most once, matching the original insert behaviour.
          path.reserve(path.size() + seg.vertices.size() - 1);
          for (size_t i = 1; i < seg.vertices.size(); ++i) {
            path.push_back(seg.vertices[i]);
          }
        }
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
  };

  // Lazy K-way merge over the Cartesian product of segment candidates, ordered
  // by the re-costed full-path cost (not the sum of per-hop costs). Start from
  // the all-best pick and expand a neighbor per segment each time a pick is
  // popped.
  auto combo_cost = [&](const std::vector<int>& pick) {
    const ShortestPath stitched = stitch(pick);
    if (!stitched.found) {
      return std::numeric_limits<double>::infinity();
    }
    return stitched.cost;
  };
  struct HeapItem {
    double cost;
    std::vector<int> pick;
    bool operator>(const HeapItem& o) const { return cost > o.cost; }
  };
  std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<>> heap;
  // Dedup queued picks by a 64-bit FNV-1a hash of the pick vector, rather than
  // copying every pick into a std::set<vector<int>>. A collision would drop one
  // combo from the merge, but stitch() validates every emitted path, so the
  // worst case is a missed alternative, never a wrong route.
  auto hash_pick = [](const std::vector<int>& pick) -> uint64_t {
    uint64_t h = 1469598103934665603ULL;  // FNV-1a offset basis
    for (int idx : pick) {
      const auto u = static_cast<uint32_t>(idx);
      for (int b = 0; b < 4; ++b) {
        h ^= static_cast<uint64_t>((u >> (b * 8)) & 0xFF);
        h *= 1099511628211ULL;  // FNV prime
      }
    }
    return h;
  };
  std::unordered_set<uint64_t> queued;

  // Lazy K-way merge: combo_cost re-stitches, so skip infinite (invalid) starts.
  std::vector<int> start(hops, 0);
  const double start_cost = combo_cost(start);
  if (!std::isfinite(start_cost)) {
    return results;
  }
  heap.push({start_cost, start});
  queued.insert(hash_pick(start));

  while (!heap.empty() && static_cast<int>(results.size()) < k) {
    const std::vector<int> pick = heap.top().pick;
    heap.pop();

    const ShortestPath stitched = stitch(pick);
    if (stitched.found) {
      results.push_back(stitched);
    }

    // Enqueue the neighbors that advance one segment's candidate index.
    for (size_t h = 0; h < hops; ++h) {
      if (pick[h] + 1 < static_cast<int>(segments[h].size())) {
        std::vector<int> next = pick;
        next[h] += 1;
        if (queued.insert(hash_pick(next)).second) {
          const double c = combo_cost(next);
          if (std::isfinite(c)) {
            heap.push({c, next});
          }
        }
      }
    }
  }

  return results;
}

}  // namespace bf
