// SPDX-License-Identifier: LGPL-3.0-or-later
#include <algorithm>
#include <cstdint>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/constraints/altitude_constraints.h"
#include "core/constraints/avoid_constraint.h"
#include "core/constraints/mora_constraint.h"
#include "core/constraints/randomize_constraint.h"
#include "core/graph/yen_kshortest.h"
#include "core/routing/route_string.h"
#include "core/util/string_util.h"
#include "io/graph_builder.h"
#include "io/nav_database.h"
#include "io/procedure_connector.h"

namespace bf {

namespace {

// How one endpoint of a query attaches to the enroute graph. An airport with
// procedures contributes several seeded connection fixes; a plain waypoint or a
// DCT-fallback airport contributes one or a few. `airport_icao` is empty for a
// bare waypoint endpoint. `has_procedures` records whether the airport actually
// publishes procedures for this side (SID for departure, STAR for arrival), so a
// DCT fallback can be told apart from missing data: procedures that exist but
// reach no on-network fix (radar vectors) still fall back to DCT.
struct EndpointPlan {
  std::vector<Connection> connections;
  std::string airport_icao;  // empty if the endpoint is a plain waypoint
  bool used_procedures = false;
  bool has_procedures = false;
  // Set when the request named a SID/STAR that the airport does not publish (or
  // whose fixes reach no on-network vertex): the caller reports an Error instead
  // of silently falling back to DCT or another procedure.
  bool named_procedure_unmatched = false;
};

// Format a procedure reference as "NAME.TRANSITION" (or just "NAME" when the
// transition is empty / the common segment).
std::string FormatRef(const ProcedureRef& ref) {
  if (ref.transition.empty()) {
    return ref.name;
  }
  return ref.name + "." + ref.transition;
}

// Whether a procedure ref matches a requested selector. The selector is either
// a bare name ("DEEZZ5", matches any transition) or "NAME.TRANSITION"
// ("DEEZZ5.TOWIN", matches that transition exactly). Comparison is
// case-sensitive (CIFP names are already upper-case).
bool RefMatchesSelector(const ProcedureRef& ref, const std::string& selector) {
  const size_t dot = selector.find('.');
  if (dot == std::string::npos) {
    return ref.name == selector;
  }
  return ref.name == selector.substr(0, dot) && ref.transition == selector.substr(dot + 1);
}

// Filter connections in place to only those procedures matching `selector`,
// dropping any connection left with no matching procedure. Returns true if at
// least one procedure survived. A no-op returning true when the selector is
// empty (no name requested).
bool FilterConnectionsByName(std::vector<Connection>& connections, const std::string& selector) {
  if (selector.empty()) {
    return true;
  }
  bool any = false;
  for (Connection& c : connections) {
    std::vector<ProcedureRef> kept;
    for (const ProcedureRef& ref : c.procedures) {
      if (RefMatchesSelector(ref, selector)) {
        kept.push_back(ref);
      }
    }
    c.procedures = std::move(kept);
    if (!c.procedures.empty()) {
      any = true;
    }
  }
  if (any) {
    // Drop connections that no longer carry any matching procedure so the search
    // only seeds fixes reachable by the requested procedure.
    std::vector<Connection> filtered;
    for (Connection& c : connections) {
      if (!c.procedures.empty()) {
        filtered.push_back(std::move(c));
      }
    }
    connections = std::move(filtered);
  }
  return any;
}

// Build a Route from a path of connection-fix vertices. `dep_label`/`arr_label`
// are the airport ICAOs to show as the true endpoints (empty for waypoint
// endpoints). When an airport endpoint connects through a procedure, `sid`/
// `star` name it and `dep_seed`/`arr_seed` are the estimated procedure
// distances; these become explicit first/last legs (airport <-> connection fix)
// and are embedded in the route string like a filed flight plan.
Route MakeRoute(const GraphBuilder& builder, const NavGraph& graph, const ShortestPath& path,
                const std::string& dep_label, const std::string& arr_label, const std::string& sid,
                const std::string& star, double dep_seed, double arr_seed,
                const SearchOptions& options) {
  Route route;
  route.total_distance_nm = path.distance_nm;
  if (path.vertices.empty()) {
    return route;
  }
  const std::string dep_fix_id = builder.IdentOf(path.vertices.front()).ident;
  const std::string arr_fix_id = builder.IdentOf(path.vertices.back()).ident;

  // Points: optional departure airport, the connection fixes along the path,
  // then the optional arrival airport.
  if (!dep_label.empty()) {
    route.points.push_back(RoutePoint{dep_label, graph.CoordOf(path.vertices.front())});
  }
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder.IdentOf(v).ident, graph.CoordOf(v)});
  }
  if (!arr_label.empty()) {
    route.points.push_back(RoutePoint{arr_label, graph.CoordOf(path.vertices.back())});
  }

  // Leading procedure leg: airport -> first connection fix. The leg's `via`
  // carries the literal connector keyword "SID" (or "DCT" when the airport fell
  // back to a direct link), not the procedure name -- the name lives in
  // route.sid, so the compact route string stays airline/ICAO style.
  if (!dep_label.empty()) {
    route.legs.push_back(
        RouteLeg{dep_label, dep_fix_id, sid.empty() ? "DCT" : "SID", dep_seed, {}});
  }
  // Enroute legs between consecutive on-network fixes.
  for (size_t i = 0; i + 1 < path.vertices.size(); ++i) {
    const int u = path.vertices[i];
    const int w = path.vertices[i + 1];
    // u->w may have parallel edges (several airways, or an airway plus a DCT).
    // Label the leg with the edge the search actually traversed -- the cheapest
    // ALLOWED edge by effective cost (distance + soft penalties) -- via the shared
    // SelectEdge helper, so the via name and distance match the path's cost and
    // total_distance_nm. Picking the shortest-by-distance edge (the previous
    // behavior) could show a via/distance the cost model did not choose when a
    // constraint (level preference, randomization) made a longer edge cheaper.
    const GraphEdge* e = SelectEdge(graph, u, w, options);
    std::string via = "DCT";
    double dist = 0.0;
    if (e != nullptr) {
      via = builder.AirwayName(e->airway_id);
      dist = e->distance_nm;
    }
    route.legs.push_back(
        RouteLeg{builder.IdentOf(u).ident, builder.IdentOf(w).ident, via, dist, {}});
  }
  // Trailing procedure leg: last connection fix -> airport. `via` carries the
  // literal "STAR" (or "DCT"); the STAR name lives in route.star.
  if (!arr_label.empty()) {
    route.legs.push_back(
        RouteLeg{arr_fix_id, arr_label, star.empty() ? "DCT" : "STAR", arr_seed, {}});
  }

  // Phase split, computed here from the leg positions (not re-derived at print
  // time). The dep/arr seeds are the procedure-leg distances when an airport
  // endpoint exists; total already includes both, so enroute is the remainder.
  route.dep_distance_nm = dep_label.empty() ? 0.0 : dep_seed;
  route.arr_distance_nm = arr_label.empty() ? 0.0 : arr_seed;
  route.enroute_distance_nm =
      route.total_distance_nm - route.dep_distance_nm - route.arr_distance_nm;

  // Route string in filed-flight-plan style: DEP SID FIX <airways> FIX STAR ARR,
  // where "SID"/"STAR" are literal connector keywords (the procedure names are in
  // route.sid/star). BuildRouteString folds consecutive legs on a shared airway
  // (listing it only at the join/leave fixes) and, as a side effect, rewrites each
  // leg's `via` to the single chosen designator and records any concurrency in
  // `concurrent_airways`.
  const std::string first_point = route.points.empty() ? "" : route.points.front().ident;
  route.route_string = BuildRouteString(first_point, route.legs);
  return route;
}

// Pick the primary procedure (name + runway) and all interchangeable options
// for the connection fix `fix_vertex` within `plan`. Returns the chosen name in
// `name`/`runway` and every "NAME.TRANSITION" sharing the fix in `options`.
void SelectProcedures(const EndpointPlan& plan, int fix_vertex, std::string& name,
                      std::string& runway, std::vector<std::string>& options) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex != fix_vertex) {
      continue;
    }
    for (const ProcedureRef& ref : c.procedures) {
      options.push_back(FormatRef(ref));
      if (name.empty()) {
        name = ref.name;
        runway = ref.runway;
      }
    }
    break;
  }
}

// Resolve the request's avoid_waypoints to the set of vertices to block. A full
// "IDENT/REGION" key resolves to that single vertex; a bare "IDENT" resolves to
// every region's match (idents are not globally unique, so "avoid X" avoids all
// X). Unknown idents contribute nothing (avoiding something absent is a no-op).
std::unordered_set<int> ResolveAvoidVertices(const GraphBuilder& builder,
                                             const std::vector<std::string>& avoid_waypoints) {
  std::unordered_set<int> out;
  for (const std::string& raw : avoid_waypoints) {
    const std::string up = ToUpper(raw);
    const size_t slash = up.find('/');
    if (slash != std::string::npos) {
      const int v = builder.VertexByIdent(Ident(up.substr(0, slash), up.substr(slash + 1)));
      if (v >= 0) {
        out.insert(v);
      }
    } else {
      for (const int v : builder.VerticesByIdent(up)) {
        out.insert(v);
      }
    }
  }
  return out;
}

// Resolve the request's avoid_airways (by designator) to the set of airway_ids
// to block. Because a stored airway name may be a concurrency ("J60-V123"), an
// airway_id is included when any of its designators is in the avoid set -- so
// avoiding "J60" also blocks segments recorded under "J60-V123".
std::unordered_set<uint16_t> ResolveAvoidAirwayIds(const GraphBuilder& builder,
                                                   const std::vector<std::string>& avoid_airways) {
  std::unordered_set<std::string> wanted;
  for (const std::string& a : avoid_airways) {
    wanted.insert(ToUpper(a));
  }
  std::unordered_set<uint16_t> out;
  if (wanted.empty()) {
    return out;
  }
  const std::vector<std::string>& names = builder.AirwayNames();
  for (size_t id = 1; id < names.size(); ++id) {  // id 0 = "DCT", never avoided
    for (const std::string& designator : SplitDesignators(names[id])) {
      if (wanted.count(designator) != 0) {
        out.insert(static_cast<uint16_t>(id));
        break;
      }
    }
  }
  return out;
}

// Resolve one forced ("via") point token to a graph vertex. A full
// "IDENT/REGION" key resolves exactly; a bare ident with several regional
// matches picks the one adding the least detour to the dep->arr great circle
// (deterministic and explainable). Airports are rejected (a via point is an
// enroute fix, and airports are barred as transit nodes anyway). On success,
// writes the resolved "IDENT/REGION" to `echo`. Returns the vertex, or -1 if no
// non-airport match exists (the caller reports an unknown-forced-point error).
int ResolveForcedPoint(const GraphBuilder& builder, const std::string& token,
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
    echo = id.ident + "/" + id.region;
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
  echo = builder.IdentOf(best).ident + "/" + builder.IdentOf(best).region;
  return best;
}

// Stitch a route through an ordered list of forced ("via") vertices. The route
// is searched in hops -- sources -> F1, Fi -> Fi+1 for each interior pair, then
// Fn -> goals -- and concatenated. Only the first hop carries the real source
// seeds and only the last the real goal seeds; interior forced vertices are
// seeded at 0 so their cost is not double counted at the seams. Every hop
// honors all constraints and node/edge bans in `options`.
//
// Up to `k` whole routes are returned, ordered by total (segment-sum) cost.
// Each hop is expanded into up to `k` alternatives via K-shortest; the best K
// end-to-end combinations are then selected by a "sum of per-segment costs"
// best-first merge over the Cartesian product (a lazy K-way merge that touches
// O(k * hops) combinations, not the full product). A combination whose stitched
// path repeats a vertex (a cycle at some seam) is skipped -- forced routing is
// order-sensitive, so a repeated fix is not a valid simple route.
//
// A returned path's distance_nm/cost include both endpoint seeds, matching the
// non-forced path so downstream MakeRoute treats them identically.
std::vector<ShortestPath> FindForcedPaths(const NavGraph& graph,
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

  // Stitch one combination (one candidate index per segment) into a full path.
  // Returns found=false if the segments do not meet or the result has a cycle.
  auto stitch = [&](const std::vector<int>& pick) -> ShortestPath {
    ShortestPath out;
    std::vector<int> path;
    double dist = 0.0;
    double cost = 0.0;
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
      dist += seg.distance_nm;
      cost += seg.cost;
    }
    std::unordered_set<int> seen;
    seen.reserve(path.size());
    for (const int v : path) {
      if (!seen.insert(v).second) {
        return out;  // cycle at a seam -> not a simple route
      }
    }
    out.vertices = std::move(path);
    out.distance_nm = dist;
    out.cost = cost;
    out.found = true;
    return out;
  };

  // Lazy K-way merge over the Cartesian product of segment candidates, ordered
  // by the sum of per-segment costs. Start from the all-best pick and expand a
  // neighbor per segment (increment one index) each time a pick is popped.
  auto combo_cost = [&](const std::vector<int>& pick) {
    double c = 0.0;
    for (size_t h = 0; h < hops; ++h) {
      c += segments[h][pick[h]].cost;
    }
    return c;
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

  std::vector<int> start(hops, 0);
  heap.push({combo_cost(start), start});
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
          heap.push({combo_cost(next), next});
        }
      }
    }
  }

  return results;
}

}  // namespace

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  // Resolve an endpoint into how it attaches to the network. An airport with
  // CIFP procedures connects through them (procedure-first); without CIFP it
  // falls back to DCT links to the nearest on-network waypoints (M1 behavior);
  // a plain waypoint connects as itself.
  auto plan_endpoint = [&](const std::string& name, bool departure) -> EndpointPlan {
    const std::string up = ToUpper(name);
    EndpointPlan plan;
    const int airport = builder_->VertexByAirport(up);
    if (airport >= 0) {
      plan.airport_icao = up;
      const Coordinate apt = builder_->graph().CoordOf(airport);
      const CifpData* cifp = ProceduresFor(up);
      if (cifp != nullptr) {
        const ProcedureType want = departure ? ProcedureType::kSid : ProcedureType::kStar;
        for (const Procedure& p : cifp->procedures) {
          if (p.type == want) {
            plan.has_procedures = true;
            break;
          }
        }
        const std::string& rwy = departure ? request.departure_runway : request.arrival_runway;
        plan.connections = departure
                               ? ProcedureConnector::BuildDeparture(*cifp, apt, *builder_, rwy)
                               : ProcedureConnector::BuildArrival(*cifp, apt, *builder_, rwy);
        // Optional SID/STAR selection by name: keep only the requested procedure.
        // If none matches, mark it so the caller errors instead of falling back.
        const std::string& sel = departure ? request.departure_sid : request.arrival_star;
        if (!sel.empty() && !FilterConnectionsByName(plan.connections, sel)) {
          plan.connections.clear();
          plan.named_procedure_unmatched = true;
          return plan;
        }
        plan.used_procedures = !plan.connections.empty();
      } else if (!(departure ? request.departure_sid : request.arrival_star).empty()) {
        // A procedure was named but the airport has no CIFP data at all.
        plan.named_procedure_unmatched = true;
        return plan;
      }
      if (plan.connections.empty()) {
        // No usable procedures: fall back to DCT links to the nearest
        // on-network waypoints. The airport stays the route endpoint; the
        // connecting leg shows "DCT" since no procedure was selected. Filter by
        // direction: a departure needs an outbound-capable fix, an arrival an
        // inbound-capable one (a STAR entry gate is often inbound-only).
        plan.connections =
            ProcedureConnector::BuildDctFallback(apt, *builder_, 5, /*arrival=*/!departure);
      }
      return plan;
    }
    // A bare ident is no longer accepted as a route endpoint: idents are not
    // globally unique, and silently picking one region's match would put the
    // whole route on the wrong endpoint. The caller must use an airport ICAO or
    // a (ident, region) pair. With no airport and no ident hit, plan.connections
    // stays empty, so the caller reports an "unknown endpoint" error.
    return plan;
  };

  EndpointPlan dep = plan_endpoint(request.departure, /*departure=*/true);
  if (dep.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "departure airport " + request.departure +
                                                              " has no SID matching '" +
                                                              request.departure_sid + "'"));
  }
  if (dep.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown departure: " + request.departure + " (use an airport ICAO code, e.g. KLAX)"));
  }
  EndpointPlan arr = plan_endpoint(request.arrival, /*departure=*/false);
  if (arr.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "arrival airport " + request.arrival +
                                                              " has no STAR matching '" +
                                                              request.arrival_star + "'"));
  }
  if (arr.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound,
              "unknown arrival: " + request.arrival + " (use an airport ICAO code, e.g. KLAX)"));
  }

  // Assemble the active constraints from the request.
  AltitudeBandConstraint altitude_band;
  MoraConstraint mora(mora_);
  LevelPreferenceConstraint level_pref;
  // Resolve avoid sets once; the constraint holds them for the whole search
  // (and every Yen spur), so it must outlive the calls below. The vertex set is
  // also used to prune seeded endpoints (below): AvoidConstraint only blocks
  // edges entering a vertex, but a source/goal fix is seeded, not entered, so an
  // avoided connection fix must be removed from the endpoint sets directly.
  const std::unordered_set<int> avoid_vertices =
      ResolveAvoidVertices(*builder_, request.avoid_waypoints);
  AvoidConstraint avoid(avoid_vertices, ResolveAvoidAirwayIds(*builder_, request.avoid_airways));
  RandomizeConstraint randomize(request.random_seed.value_or(0));
  SearchOptions options;
  options.request = &request;
  if (request.altitude.has_value()) {
    options.constraints.push_back(&altitude_band);
    options.constraints.push_back(&mora);
  }
  if (request.level != LevelPreference::kNone) {
    options.constraints.push_back(&level_pref);
  }
  if (!request.avoid_waypoints.empty() || !request.avoid_airways.empty()) {
    options.constraints.push_back(&avoid);
  }
  if (request.random_seed.has_value()) {
    options.constraints.push_back(&randomize);
  }
  // Airports must not be transit nodes: their synthetic DCT links would let the
  // search cut through an unrelated airport (e.g. ...MIE DCT KMIE SNKPT...).
  // Endpoints connect via seeded connection fixes, not airport vertices, so
  // blocking all airport vertices as intermediate nodes is safe. Airports occupy
  // the contiguous tail [first_airport_vertex, VertexCount), so a NodeFilter
  // range check replaces the old IsAirport std::function -- an inlined
  // two-compare on the hot loop instead of a type-erased call per neighbor.
  const NavGraph& graph = builder_->graph();
  options.node_filter = NodeFilter{builder_->first_airport_vertex(), graph.VertexCount(), nullptr};
  std::vector<SeededEndpoint> sources = ProcedureConnector::ToEndpoints(dep.connections);
  std::vector<SeededEndpoint> goals = ProcedureConnector::ToEndpoints(arr.connections);
  // Drop any seeded connection fix the request asks to avoid: it would otherwise
  // slip through as a search start/end, which AvoidConstraint cannot catch.
  if (!avoid_vertices.empty()) {
    auto drop_avoided = [&](std::vector<SeededEndpoint>& eps) {
      eps.erase(std::remove_if(
                    eps.begin(), eps.end(),
                    [&](const SeededEndpoint& e) { return avoid_vertices.count(e.vertex) != 0; }),
                eps.end());
    };
    drop_avoided(sources);
    drop_avoided(goals);
    if (sources.empty() || goals.empty()) {
      return Result<Routes>::Err(
          Error(ErrorCode::kNoRoute, "no route between endpoints (avoided all connection fixes)"));
    }
  }

  // Resolve forced ("via") points to an ordered vertex list. Disambiguation of a
  // bare ident uses the dep->arr great circle: pick the match adding the least
  // detour. Endpoint coordinates come from the airport vertex when there is one,
  // else from the first seeded connection fix.
  std::vector<int> forced;
  std::vector<std::string> forced_echo;
  if (!request.forced_points.empty()) {
    const int dep_apt = builder_->VertexByAirport(ToUpper(request.departure));
    const int arr_apt = builder_->VertexByAirport(ToUpper(request.arrival));
    const Coordinate dep_coord =
        dep_apt >= 0 ? graph.CoordOf(dep_apt) : graph.CoordOf(sources.front().vertex);
    const Coordinate arr_coord =
        arr_apt >= 0 ? graph.CoordOf(arr_apt) : graph.CoordOf(goals.front().vertex);
    forced.reserve(request.forced_points.size());
    forced_echo.reserve(request.forced_points.size());
    for (const std::string& token : request.forced_points) {
      std::string echo;
      bool is_airport = false;
      const int v = ResolveForcedPoint(*builder_, token, dep_coord, arr_coord, echo, is_airport);
      if (v < 0) {
        const std::string why =
            is_airport ? "' is an airport, not an enroute waypoint" : "' is not a known waypoint";
        return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "forced point '" + token + why));
      }
      if (avoid_vertices.count(v) != 0) {
        return Result<Routes>::Err(
            Error(ErrorCode::kNoRoute, "forced point '" + token + "' is also in the avoid list"));
      }
      forced.push_back(v);
      forced_echo.push_back(echo);
    }
  }

  // Find up to k candidate routes. Unlike the earlier scheme that fixed a single
  // best connection-fix pair and only varied the enroute portion between them,
  // the multi-endpoint Yen lets each candidate join through a different SID/STAR
  // connection fix, so the alternatives can use genuinely different procedures.
  // Both forms report distance_nm with both seed costs already included.
  const int k = std::max(1, request.k);
  std::vector<ShortestPath> paths;
  if (!forced.empty()) {
    // Forced points: search each hop (sources -> F1 -> ... -> Fn -> goals) with
    // K-shortest and merge the best end-to-end combinations. Returns up to k
    // whole routes through the forced points, in cost order.
    paths = FindForcedPaths(graph, sources, goals, forced, k, options);
  } else if (k == 1) {
    const ShortestPath best = FindShortestPathMulti(graph, sources, goals, options);
    if (best.found && !best.vertices.empty()) {
      paths.push_back(best);
    }
  } else {
    paths = FindKShortestPathsMulti(graph, sources, goals, k, options);
  }
  if (paths.empty()) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "no route between endpoints"));
  }

  // Look up a connection fix's seed cost among an endpoint's seeded fixes.
  auto seed_of = [](const std::vector<SeededEndpoint>& eps, int vertex) {
    for (const SeededEndpoint& e : eps) {
      if (e.vertex == vertex) {
        return e.cost;
      }
    }
    return 0.0;
  };

  // Classify how an endpoint attached to the network. A plan's connections are
  // homogeneous (all from a procedure build, or all DCT fallback), so this is a
  // per-endpoint verdict: a procedure was used; else procedures exist but none
  // reached the network (radar vectors); else no procedure data at all.
  auto connection_kind = [](const EndpointPlan& plan) {
    if (plan.used_procedures) {
      return ConnectionKind::kProcedure;
    }
    if (plan.has_procedures) {
      return ConnectionKind::kRadarVectors;
    }
    return ConnectionKind::kDirect;
  };
  const ConnectionKind dep_kind = connection_kind(dep);
  const ConnectionKind arr_kind = connection_kind(arr);

  Routes routes;
  routes.reserve(paths.size());
  for (const ShortestPath& p : paths) {
    if (p.vertices.empty()) {
      continue;
    }
    const int dep_fix = p.vertices.front();
    const int arr_fix = p.vertices.back();
    const double dep_seed = seed_of(sources, dep_fix);
    const double arr_seed = seed_of(goals, arr_fix);

    // Procedure selection depends on the candidate's own fix pair, which may
    // differ across candidates, so resolve it per path.
    std::string sid_name;
    std::string dep_rwy;
    std::vector<std::string> sid_options;
    SelectProcedures(dep, dep_fix, sid_name, dep_rwy, sid_options);
    std::string star_name;
    std::string arr_rwy;
    std::vector<std::string> star_options;
    SelectProcedures(arr, arr_fix, star_name, arr_rwy, star_options);

    Route route = MakeRoute(*builder_, graph, p, dep.airport_icao, arr.airport_icao, sid_name,
                            star_name, dep_seed, arr_seed, options);
    route.sid = sid_name;
    route.dep_runway = dep_rwy;
    route.sid_options = sid_options;
    route.star = star_name;
    route.arr_runway = arr_rwy;
    route.star_options = star_options;
    route.dep_connection = dep_kind;
    route.arr_connection = arr_kind;
    route.forced_points = forced_echo;
    routes.push_back(std::move(route));
  }
  return Result<Routes>::Ok(std::move(routes));
}

}  // namespace bf
