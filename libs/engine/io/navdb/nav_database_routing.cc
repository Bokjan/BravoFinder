// SPDX-License-Identifier: LGPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/constraints/airway_rule_constraint.h"
#include "core/constraints/altitude_constraints.h"
#include "core/constraints/avoid_waypoint_constraint.h"
#include "core/constraints/mora_constraint.h"
#include "core/constraints/randomize_constraint.h"
#include "core/domain/nav_tokens.h"
#include "core/graph/yen_kshortest.h"
#include "core/routing/route_string.h"
#include "io/build/graph_builder.h"
#include "io/build/procedure_connector.h"
#include "io/navdb/nav_database.h"

namespace bf {

namespace {

// How one endpoint of a query attaches to the enroute graph. An airport with
// procedures contributes several seeded connection fixes; a DCT-fallback
// airport contributes one or a few. `airport_icao` is always set for a
// successful plan (FindRoutes only accepts airport ICAO endpoints).
// `has_procedures` records whether the airport actually publishes procedures
// for this side (SID for departure; STAR or approach for arrival), so a DCT
// fallback can be told apart from missing data: procedures that exist but
// reach no on-network fix (radar vectors) still fall back to DCT.
struct EndpointPlan {
  std::vector<Connection> connections;
  std::string airport_icao;  // empty if the endpoint is a plain waypoint
  bool used_procedures = false;
  bool used_approach = false;
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

// Soft-prefer STAR splice over approach when both compete in the same arrival
// pool (issue #30). Applied to search ranking / SeededEndpoint.cost only — never
// written into Connection::seed_distance_nm (which stays geographic for MakeRoute
// and reported distances). Pure no-STAR (#24) airports must not pay this bump.
constexpr double kProcedurePreferNm = 15.0;

const Connection* FindConnection(const EndpointPlan& plan, int fix_vertex) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex == fix_vertex) {
      return &c;
    }
  }
  return nullptr;
}

bool IsApproachFront(const Connection& c) {
  return !c.procedures.empty() && c.procedures.front().type == ProcedureType::kApproach;
}

// True when the arrival pool still holds both STAR and approach refs (soft-prefer
// was armed at build time and both sides survived Accumulate).
bool SoftPreferStarActive(const EndpointPlan& plan) {
  bool any_star = false;
  bool any_apch = false;
  for (const Connection& c : plan.connections) {
    for (const ProcedureRef& ref : c.procedures) {
      if (ref.type == ProcedureType::kApproach) {
        any_apch = true;
      } else if (ref.type == ProcedureType::kStar) {
        any_star = true;
      }
    }
  }
  return any_star && any_apch;
}

double EffectiveSeedNm(const Connection& c, bool soft_prefer_star) {
  double seed = c.seed_distance_nm;
  if (soft_prefer_star && IsApproachFront(c)) {
    seed += kProcedurePreferNm;
  }
  return seed;
}

// Fold `incoming` into `by_fix` by fix_vertex. Ranking uses effective seed
// (approach + B when soft-prefer is on); stored seed_distance_nm stays raw.
void MergeConnection(std::vector<Connection>& by_fix, Connection incoming, bool soft_prefer_star) {
  for (Connection& c : by_fix) {
    if (c.fix_vertex != incoming.fix_vertex) {
      continue;
    }
    const size_t first_incoming = c.procedures.size();
    for (ProcedureRef& ref : incoming.procedures) {
      c.procedures.push_back(std::move(ref));
    }
    if (EffectiveSeedNm(incoming, soft_prefer_star) < EffectiveSeedNm(c, soft_prefer_star)) {
      c.seed_distance_nm = incoming.seed_distance_nm;
      c.bearing = incoming.bearing;
      c.approach_bearing = incoming.approach_bearing;
      c.splice_vertex = incoming.splice_vertex;
      c.splice_leg_nm = incoming.splice_leg_nm;
      if (first_incoming < c.procedures.size()) {
        std::swap(c.procedures.front(), c.procedures[first_incoming]);
      }
    }
    return;
  }
  by_fix.push_back(std::move(incoming));
}

void MergeAll(std::vector<Connection>& dst, std::vector<Connection> src, bool soft_prefer_star) {
  for (Connection& c : src) {
    MergeConnection(dst, std::move(c), soft_prefer_star);
  }
}

void SortConnectionsBySeed(std::vector<Connection>& by_fix, bool soft_prefer_star) {
  std::sort(by_fix.begin(), by_fix.end(),
            [soft_prefer_star](const Connection& a, const Connection& b) {
              const double sa = EffectiveSeedNm(a, soft_prefer_star);
              const double sb = EffectiveSeedNm(b, soft_prefer_star);
              if (sa != sb) {
                return sa < sb;
              }
              return a.fix_vertex < b.fix_vertex;
            });
}

std::vector<SeededEndpoint> ToSearchEndpoints(const std::vector<Connection>& connections,
                                              bool soft_prefer_star) {
  std::vector<SeededEndpoint> endpoints;
  endpoints.reserve(connections.size());
  for (const Connection& c : connections) {
    endpoints.push_back(
        SeededEndpoint{c.fix_vertex, EffectiveSeedNm(c, soft_prefer_star), c.bearing});
  }
  return endpoints;
}

Route MakeRoute(const GraphBuilder& builder, const NavGraph& graph, const ShortestPath& path,
                const std::string& dep_label, const std::string& arr_label, const std::string& sid,
                const std::string& star, double dep_seed, double arr_seed,
                const SearchOptions& options, int dep_splice_vertex = kNoVertex,
                double dep_splice_leg_nm = 0.0, int arr_splice_vertex = kNoVertex,
                double arr_splice_leg_nm = 0.0) {
  Route route;
  route.total_distance_nm = path.distance_nm;
  if (path.vertices.empty()) {
    return route;
  }
  const std::string dep_fix_id = builder.IdentOf(path.vertices.front()).ident;
  const std::string arr_fix_id = builder.IdentOf(path.vertices.back()).ident;
  const bool dep_splice = dep_splice_vertex >= 0 && !dep_label.empty();
  const bool arr_splice = arr_splice_vertex >= 0 && !arr_label.empty();
  const double dep_body_nm = dep_splice ? std::max(0.0, dep_seed - dep_splice_leg_nm) : dep_seed;
  const double arr_body_nm = arr_splice ? std::max(0.0, arr_seed - arr_splice_leg_nm) : arr_seed;
  const std::string dep_exit_id =
      dep_splice ? builder.IdentOf(dep_splice_vertex).ident : std::string{};
  const std::string arr_entry_id =
      arr_splice ? builder.IdentOf(arr_splice_vertex).ident : std::string{};

  // Points: optional departure airport, optional SID exit (splice), the
  // connection fixes along the path, optional STAR entry (splice), then the
  // optional arrival airport.
  if (!dep_label.empty()) {
    const int dep_apt = builder.VertexByAirport(ToUpper(dep_label));
    const Coordinate dep_coord =
        dep_apt >= 0 ? graph.CoordOf(dep_apt) : graph.CoordOf(path.vertices.front());
    route.points.push_back(RoutePoint{dep_label, dep_coord});
    if (dep_splice) {
      route.points.push_back(RoutePoint{dep_exit_id, graph.CoordOf(dep_splice_vertex)});
    }
  }
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder.IdentOf(v).ident, graph.CoordOf(v)});
  }
  if (!arr_label.empty()) {
    if (arr_splice) {
      route.points.push_back(RoutePoint{arr_entry_id, graph.CoordOf(arr_splice_vertex)});
    }
    const int arr_apt = builder.VertexByAirport(ToUpper(arr_label));
    const Coordinate arr_coord =
        arr_apt >= 0 ? graph.CoordOf(arr_apt) : graph.CoordOf(path.vertices.back());
    route.points.push_back(RoutePoint{arr_label, arr_coord});
  }

  // Leading procedure leg(s): airport -> [exit DCT] -> first connection fix.
  if (!dep_label.empty()) {
    if (dep_splice) {
      route.legs.push_back(
          RouteLeg{dep_label, dep_exit_id, std::string(kSidToken), dep_body_nm, {}});
      route.legs.push_back(
          RouteLeg{dep_exit_id, dep_fix_id, std::string(kDctToken), dep_splice_leg_nm, {}});
    } else {
      route.legs.push_back(RouteLeg{dep_label,
                                    dep_fix_id,
                                    sid.empty() ? std::string(kDctToken) : std::string(kSidToken),
                                    dep_seed,
                                    {}});
    }
  }
  // Enroute legs between consecutive on-network fixes.
  for (size_t i = 0; i + 1 < path.vertices.size(); ++i) {
    const int u = path.vertices[i];
    const int w = path.vertices[i + 1];
    const GraphEdge* e = SelectEdge(graph, u, w, options);
    std::string via(kDctToken);
    double dist = 0.0;
    if (e != nullptr) {
      via = builder.AirwayName(e->airway_id);
      dist = e->distance_nm;
    }
    route.legs.push_back(
        RouteLeg{builder.IdentOf(u).ident, builder.IdentOf(w).ident, via, dist, {}});
  }
  // Trailing procedure leg(s): last connection fix -> [entry STAR] -> airport.
  if (!arr_label.empty()) {
    if (arr_splice) {
      route.legs.push_back(
          RouteLeg{arr_fix_id, arr_entry_id, std::string(kDctToken), arr_splice_leg_nm, {}});
      route.legs.push_back(
          RouteLeg{arr_entry_id, arr_label, std::string(kStarToken), arr_body_nm, {}});
    } else {
      route.legs.push_back(RouteLeg{arr_fix_id,
                                    arr_label,
                                    star.empty() ? std::string(kDctToken) : std::string(kStarToken),
                                    arr_seed,
                                    {}});
    }
  }

  // Phase split (D9): dep/arr are literal SID/STAR leg distances; DCT splice
  // falls into enroute. Search seeds still priced the full |F→gate|+body.
  route.dep_distance_nm = dep_label.empty() ? 0.0 : dep_body_nm;
  route.arr_distance_nm = arr_label.empty() ? 0.0 : arr_body_nm;
  route.enroute_distance_nm =
      std::max(0.0, route.total_distance_nm - route.dep_distance_nm - route.arr_distance_nm);

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
      // Approach refs are not STARs — handled by SelectApproachProcedures.
      if (ref.type == ProcedureType::kApproach) {
        continue;
      }
      options.push_back(FormatRef(ref));
      if (name.empty()) {
        name = ref.name;
        runway = ref.runway;
      }
    }
    break;
  }
}

// Fill approach_* metadata for a terminal-transition arrival at `fix_vertex`.
void SelectApproachProcedures(const EndpointPlan& plan, int fix_vertex, std::string& approach,
                              std::string& approach_iaf, double& approach_bearing,
                              std::string& runway, std::vector<std::string>& options) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex != fix_vertex) {
      continue;
    }
    approach_bearing = c.approach_bearing;
    for (const ProcedureRef& ref : c.procedures) {
      if (ref.type != ProcedureType::kApproach) {
        continue;
      }
      options.push_back(FormatRef(ref));
      if (approach.empty()) {
        approach = FormatRef(ref);
        approach_iaf = ref.iaf;
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
std::vector<int> ResolveAvoidVertices(const GraphBuilder& builder,
                                      const std::vector<std::string>& avoid_waypoints) {
  std::vector<int> out;
  for (const std::string& raw : avoid_waypoints) {
    const std::string up = ToUpper(raw);
    const size_t slash = up.find('/');
    if (slash != std::string::npos) {
      const int v = builder.VertexByIdent(Ident(up.substr(0, slash), up.substr(slash + 1)));
      if (v >= 0) {
        out.push_back(v);
      }
    } else {
      for (const int v : builder.VerticesByIdent(up)) {
        out.push_back(v);
      }
    }
  }
  // Sort + unique so the constraint's binary_search (and the endpoint-pruning
  // binary_search at the call site) see a sorted, deduped set.
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

// Resolve the request's airway_rules into the two bitmask tables
// AirwayRuleConstraint evaluates on the hot path. Bit i of both tables is rule
// `rules[i]`, so a leg matches rule i iff its region bit and its designator bit
// are both set.
//
// All name matching happens here, once per query, so the search itself only does
// integer work -- the same discipline ResolveAvoidVertices follows. Both sides of
// a rule are lists that share the rule's single bit, so enumerating ten regions or
// two hundred designators costs nothing extra at runtime.
//
// Caller must have checked rules.size() <= kMaxRules.
AirwayRuleConstraint ResolveAirwayRules(const GraphBuilder& builder,
                                        const std::vector<AirwayRule>& rules) {
  const size_t rule_count = rules.size();
  uint32_t block_bits = 0;
  std::vector<double> fractions(rule_count, 0.0);
  for (size_t i = 0; i < rule_count; ++i) {
    if (rules[i].action == AirwayRule::Action::kBlock) {
      block_bits |= uint32_t{1} << i;
    } else {
      // Negative soft penalties break A* admissibility; clamp at the assembly
      // boundary so a hand-built RouteRequest cannot inject them.
      const double frac = rules[i].penalty_fraction;
      fractions[i] = frac < 0.0 ? 0.0 : frac;
    }
  }

  // Per-vertex region mask. Regions are drawn from a small fixed alphabet (241
  // distinct codes in AIRAC 2601) while V is ~275k, so match each DISTINCT region
  // once into a small map and then fill the big array by lookup, rather than
  // re-running the prefix comparisons per vertex.
  const int vcount = builder.graph().VertexCount();
  std::unordered_map<std::string_view, uint32_t> region_masks;
  std::vector<uint32_t> vertex_mask(static_cast<size_t>(vcount), 0);
  for (int v = 0; v < vcount; ++v) {
    const std::string_view region = builder.RegionOf(v);
    auto it = region_masks.find(region);
    if (it == region_masks.end()) {
      uint32_t mask = 0;
      for (size_t i = 0; i < rule_count; ++i) {
        if (MatchesAnyPrefix(region, rules[i].region_prefixes)) {
          mask |= uint32_t{1} << i;
        }
      }
      // The key is a view into the vertex's FixedIdent, which lives in the
      // builder for the whole query, so it stays valid for this map's lifetime.
      it = region_masks.emplace(region, mask).first;
    }
    vertex_mask[static_cast<size_t>(v)] = it->second;
  }

  // Per-airway-id designator mask. Entry 0 is the reserved "DCT" name and stays 0,
  // so synthetic edges are never subject to a rule (matching how the airway avoid
  // resolver skipped id 0). A stored name may be a concurrency ("A14-M1"), so it
  // is split into designators first and the id matches if ANY of them does.
  const std::vector<std::string>& names = builder.AirwayNames();
  std::vector<uint32_t> airway_mask(names.size(), 0);
  for (size_t id = 1; id < names.size(); ++id) {
    uint32_t mask = 0;
    for (const std::string& designator : SplitDesignators(names[id])) {
      for (size_t i = 0; i < rule_count; ++i) {
        if (MatchesAnyDesignator(designator, rules[i].designators, rules[i].match)) {
          mask |= uint32_t{1} << i;
        }
      }
    }
    airway_mask[id] = mask;
  }

  return AirwayRuleConstraint(std::move(vertex_mask), std::move(airway_mask), block_bits,
                              std::move(fractions));
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

}  // namespace

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  // Resolve an airport endpoint into how it attaches to the network. With CIFP
  // procedures: procedure-first connections; without CIFP: DCT links to the
  // nearest on-network waypoints. Non-airport tokens leave connections empty
  // so the caller reports an unknown-airport error (waypoint endpoints are not
  // supported — idents are not globally unique).
  auto plan_endpoint = [&](const std::string& name, bool departure) -> Result<EndpointPlan> {
    const std::string up = ToUpper(name);
    EndpointPlan plan;
    const int airport = builder_->VertexByAirport(up);
    if (airport >= 0) {
      plan.airport_icao = up;
      const Coordinate apt = builder_->graph().CoordOf(airport);
      Result<const CifpData*> cifp_r = ProceduresFor(up);
      if (!cifp_r) {
        return Result<EndpointPlan>::Err(std::move(cifp_r).error());
      }
      const CifpData* cifp = cifp_r.value();
      if (cifp != nullptr) {
        for (const Procedure& p : cifp->procedures) {
          if (departure) {
            if (p.type == ProcedureType::kSid) {
              plan.has_procedures = true;
              break;
            }
          } else if (p.type == ProcedureType::kStar || p.type == ProcedureType::kApproach) {
            plan.has_procedures = true;
            break;
          }
        }
        const std::string& rwy = departure ? request.departure_runway : request.arrival_runway;
        const std::string& sel = departure ? request.departure_sid : request.arrival_star;
        if (departure) {
          plan.connections = ProcedureConnector::BuildDeparture(*cifp, apt, *builder_, rwy);
          if (!sel.empty()) {
            if (!FilterConnectionsByName(plan.connections, sel)) {
              // Named --sid: only accumulate matching off-network exit splices so
              // splice_vertex/seed bind to the requested SID, not a merge winner.
              plan.connections =
                  ProcedureConnector::BuildSidSpliceDeparture(*cifp, apt, *builder_, rwy, sel);
              if (plan.connections.empty()) {
                plan.named_procedure_unmatched = true;
                return Result<EndpointPlan>::Ok(std::move(plan));
              }
            }
          } else if (plan.connections.empty()) {
            plan.connections =
                ProcedureConnector::BuildSidSpliceDeparture(*cifp, apt, *builder_, rwy);
          }
          plan.used_procedures = !plan.connections.empty();
        } else {
          plan.connections = ProcedureConnector::BuildArrival(*cifp, apt, *builder_, rwy);
          // Named --star must match a STAR connection (on-net or splice); never
          // silently fall through to approach IAFs (D1 / #30).
          if (!sel.empty()) {
            if (!FilterConnectionsByName(plan.connections, sel)) {
              plan.connections =
                  ProcedureConnector::BuildStarSpliceArrival(*cifp, apt, *builder_, rwy, sel);
              if (plan.connections.empty()) {
                plan.named_procedure_unmatched = true;
                return Result<EndpointPlan>::Ok(std::move(plan));
              }
            }
            plan.used_procedures = true;
          } else if (!plan.connections.empty()) {
            // On-network published STAR gates — today's path; do not mix approach.
            plan.used_procedures = true;
          } else {
            // Off-network published gates (or no STAR): STAR splice ∪ approach
            // in one pool. Soft-prefer STAR only when splice candidates exist.
            std::vector<Connection> pool =
                ProcedureConnector::BuildStarSpliceArrival(*cifp, apt, *builder_, rwy);
            std::vector<Connection> apch =
                ProcedureConnector::BuildApproachArrival(*cifp, apt, *builder_, rwy);
            const bool soft_prefer = !pool.empty() && !apch.empty();
            // STAR first, then approach: equal effective seeds keep STAR at front.
            MergeAll(pool, std::move(apch), soft_prefer);
            SortConnectionsBySeed(pool, soft_prefer);
            plan.connections = std::move(pool);
            // Plan-level flags are only meaningful for homogeneous pools; mixed
            // STAR∪approach is classified per path from procedures.front().type.
            bool any_star = false;
            bool any_apch = false;
            for (const Connection& c : plan.connections) {
              for (const ProcedureRef& ref : c.procedures) {
                if (ref.type == ProcedureType::kApproach) {
                  any_apch = true;
                } else if (ref.type == ProcedureType::kStar) {
                  any_star = true;
                }
              }
            }
            plan.used_procedures = any_star && !any_apch;
            plan.used_approach = any_apch && !any_star;
          }
        }
      } else if (!(departure ? request.departure_sid : request.arrival_star).empty()) {
        // A procedure was named but the airport has no CIFP data at all.
        plan.named_procedure_unmatched = true;
        return Result<EndpointPlan>::Ok(std::move(plan));
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
      return Result<EndpointPlan>::Ok(std::move(plan));
    }
    // Not an airport ICAO: leave connections empty; the caller reports unknown
    // departure/arrival. Waypoint / IDENT/REGION endpoints are intentionally
    // unsupported (idents are not globally unique).
    return Result<EndpointPlan>::Ok(std::move(plan));
  };

  Result<EndpointPlan> dep_r = plan_endpoint(request.departure, /*departure=*/true);
  if (!dep_r) {
    return Result<Routes>::Err(std::move(dep_r).error());
  }
  EndpointPlan dep = std::move(dep_r).value();
  if (dep.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kProcedureNotFound,
                                     "departure airport " + request.departure +
                                         " has no SID matching '" + request.departure_sid + "'"));
  }
  if (dep.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown departure airport: " + request.departure + " (expected an ICAO code, e.g. KLAX)"));
  }
  Result<EndpointPlan> arr_r = plan_endpoint(request.arrival, /*departure=*/false);
  if (!arr_r) {
    return Result<Routes>::Err(std::move(arr_r).error());
  }
  EndpointPlan arr = std::move(arr_r).value();
  if (arr.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kProcedureNotFound,
                                     "arrival airport " + request.arrival +
                                         " has no STAR matching '" + request.arrival_star + "'"));
  }
  if (arr.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown arrival airport: " + request.arrival + " (expected an ICAO code, e.g. KLAX)"));
  }

  // Assemble the active constraints from the request.
  AltitudeBandConstraint altitude_band;
  MoraConstraint mora(mora_);
  LevelPreferenceConstraint level_pref;
  // Resolve the avoid set once; the constraint holds it for the whole search (and
  // every Yen spur), so it must outlive the calls below. The vertex set is also
  // used to prune seeded endpoints (below): AvoidWaypointConstraint only blocks
  // edges entering a vertex, but a source/goal fix is seeded, not entered, so an
  // avoided connection fix must be removed from the endpoint sets directly.
  const std::vector<int> avoid_vertices = ResolveAvoidVertices(*builder_, request.avoid_waypoints);
  AvoidWaypointConstraint avoid(avoid_vertices);
  // Region + designator airway rules. The resolver walks the graph once here and
  // the constraint holds the resulting masks for the whole search. Held in an
  // optional so the (common) no-rules case allocates nothing; airway rules need no
  // endpoint pruning, since they match edges rather than vertices.
  std::optional<AirwayRuleConstraint> airway_rules;
  if (!request.airway_rules.empty()) {
    if (request.airway_rules.size() > AirwayRuleConstraint::kMaxRules) {
      return Result<Routes>::Err(
          Error(ErrorCode::kRouteParseError,
                std::format("too many airway rules (max {}); note one rule may list any number of "
                            "regions and designators",
                            AirwayRuleConstraint::kMaxRules)));
    }
    airway_rules.emplace(ResolveAirwayRules(*builder_, request.airway_rules));
  }
  RandomizeConstraint randomize(request.random_seed.value_or(0));
  SearchOptions options;
  options.request = &request;
  // Soft turn-angle penalty at every path vertex: suppresses the
  // near-180-degree reversals at SID/STAR handoff fixes. Always on for routing;
  // the calibration lives in the TurnPenalty constants (tunable there).
  options.turn_penalty.enabled = true;
  if (request.altitude.has_value()) {
    options.constraints.push_back(&altitude_band);
    options.constraints.push_back(&mora);
  }
  if (request.level != LevelPreference::kNone) {
    options.constraints.push_back(&level_pref);
  }
  if (!request.avoid_waypoints.empty()) {
    options.constraints.push_back(&avoid);
  }
  if (airway_rules.has_value()) {
    options.constraints.push_back(&*airway_rules);
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
  std::vector<SeededEndpoint> sources =
      ToSearchEndpoints(dep.connections, /*soft_prefer_star=*/false);
  const bool arr_soft_prefer = SoftPreferStarActive(arr);
  std::vector<SeededEndpoint> goals = ToSearchEndpoints(arr.connections, arr_soft_prefer);
  // Drop any seeded connection fix the request asks to avoid: it would otherwise
  // slip through as a search start/end, which AvoidConstraint cannot catch.
  if (!avoid_vertices.empty()) {
    auto drop_avoided = [&](std::vector<SeededEndpoint>& eps) {
      eps.erase(std::remove_if(eps.begin(), eps.end(),
                               [&](const SeededEndpoint& e) {
                                 return std::binary_search(avoid_vertices.begin(),
                                                           avoid_vertices.end(), e.vertex);
                               }),
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
        return Result<Routes>::Err(
            Error(ErrorCode::kRouteParseError, "forced point '" + token + why));
      }
      if (std::binary_search(avoid_vertices.begin(), avoid_vertices.end(), v)) {
        return Result<Routes>::Err(Error(ErrorCode::kRouteParseError,
                                         "forced point '" + token + "' is also in the avoid list"));
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
  // Geographic seed for MakeRoute / phase split — never the soft-prefer bump.
  auto raw_seed_of = [](const EndpointPlan& plan, int vertex) {
    const Connection* c = FindConnection(plan, vertex);
    return c != nullptr ? c->seed_distance_nm : 0.0;
  };

  // Classify how an endpoint attached to the network. Departure plans stay
  // homogeneous (SID or DCT). Arrival may mix STAR splice with approach in one
  // pool (#30) — those paths must be classified from the winning Connection's
  // procedures.front().type, not from plan-level used_* flags.
  auto connection_kind = [](const EndpointPlan& plan) {
    if (plan.used_approach) {
      return ConnectionKind::kTerminalTransition;
    }
    if (plan.used_procedures) {
      return ConnectionKind::kProcedure;
    }
    if (plan.has_procedures) {
      return ConnectionKind::kRadarVectors;
    }
    return ConnectionKind::kDirect;
  };
  auto arrival_kind_for_fix = [&](int arr_fix) {
    const Connection* c = FindConnection(arr, arr_fix);
    if (c != nullptr && !c->procedures.empty()) {
      if (c->procedures.front().type == ProcedureType::kApproach) {
        return ConnectionKind::kTerminalTransition;
      }
      return ConnectionKind::kProcedure;
    }
    return connection_kind(arr);
  };
  const ConnectionKind dep_kind = connection_kind(dep);

  Routes routes;
  routes.reserve(paths.size());
  for (const ShortestPath& p : paths) {
    if (p.vertices.empty()) {
      continue;
    }
    const int dep_fix = p.vertices.front();
    const int arr_fix = p.vertices.back();
    const double dep_seed = raw_seed_of(dep, dep_fix);
    const double arr_seed = raw_seed_of(arr, arr_fix);
    const Connection* dep_conn = FindConnection(dep, dep_fix);
    const Connection* arr_conn = FindConnection(arr, arr_fix);
    const int dep_splice_v = dep_conn != nullptr ? dep_conn->splice_vertex : kNoVertex;
    const double dep_splice_nm = dep_conn != nullptr ? dep_conn->splice_leg_nm : 0.0;
    const int arr_splice_v = arr_conn != nullptr ? arr_conn->splice_vertex : kNoVertex;
    const double arr_splice_nm = arr_conn != nullptr ? arr_conn->splice_leg_nm : 0.0;

    // Procedure selection depends on the candidate's own fix pair, which may
    // differ across candidates, so resolve it per path.
    std::string sid_name;
    std::string dep_rwy;
    std::vector<std::string> sid_options;
    SelectProcedures(dep, dep_fix, sid_name, dep_rwy, sid_options);
    std::string star_name;
    std::string arr_rwy;
    std::vector<std::string> star_options;
    std::string approach;
    std::string approach_iaf;
    double approach_bearing = -1.0;
    std::vector<std::string> approach_options;
    const ConnectionKind arr_kind = arrival_kind_for_fix(arr_fix);
    if (arr_kind == ConnectionKind::kTerminalTransition) {
      SelectApproachProcedures(arr, arr_fix, approach, approach_iaf, approach_bearing, arr_rwy,
                               approach_options);
    } else {
      SelectProcedures(arr, arr_fix, star_name, arr_rwy, star_options);
    }

    Route route = MakeRoute(*builder_, graph, p, dep.airport_icao, arr.airport_icao, sid_name,
                            star_name, dep_seed, arr_seed, options, dep_splice_v, dep_splice_nm,
                            arr_splice_v, arr_splice_nm);
    // A* folds SeededEndpoint.cost into distance_nm; strip the soft-prefer bump
    // so reported totals stay geographic when an approach goal won the mixed pool.
    if (arr_soft_prefer && arr_kind == ConnectionKind::kTerminalTransition) {
      route.total_distance_nm = std::max(0.0, route.total_distance_nm - kProcedurePreferNm);
      route.enroute_distance_nm =
          std::max(0.0, route.total_distance_nm - route.dep_distance_nm - route.arr_distance_nm);
    }
    route.sid = sid_name;
    route.dep_runway = dep_rwy;
    route.sid_options = sid_options;
    route.star = star_name;
    route.arr_runway = arr_rwy;
    route.star_options = star_options;
    route.dep_connection = dep_kind;
    route.arr_connection = arr_kind;
    if (arr_kind == ConnectionKind::kTerminalTransition) {
      route.terminal_transition = true;
      route.approach = std::move(approach);
      route.approach_iaf = std::move(approach_iaf);
      route.approach_bearing = approach_bearing;
      route.approach_options = std::move(approach_options);
    }
    route.forced_points = forced_echo;
    routes.push_back(std::move(route));
  }
  return Result<Routes>::Ok(std::move(routes));
}

}  // namespace bf
