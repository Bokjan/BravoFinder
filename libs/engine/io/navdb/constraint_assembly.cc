// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/navdb/constraint_assembly.h"

#include <algorithm>
#include <cstdint>
#include <format>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/domain/ident.h"
#include "core/routing/route_request.h"
#include "core/routing/route_string.h"
#include "io/build/graph_builder.h"

namespace bf {
namespace {

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

}  // namespace

Result<ConstraintBundle> ConstraintAssembly::Build(const RouteRequest& request,
                                                   const GraphBuilder& builder,
                                                   const MoraGrid& mora_grid) {
  ConstraintBundle bundle;
  bundle.avoid_vertices = ResolveAvoidVertices(builder, request.avoid_waypoints);

  if (!request.airway_rules.empty()) {
    if (request.airway_rules.size() > AirwayRuleConstraint::kMaxRules) {
      return Result<ConstraintBundle>::Err(
          Error(ErrorCode::kRouteParseError,
                std::format("too many airway rules (max {}); note one rule may list any number of "
                            "regions and designators",
                            AirwayRuleConstraint::kMaxRules)));
    }
    bundle.airway_rules =
        std::make_unique<AirwayRuleConstraint>(ResolveAirwayRules(builder, request.airway_rules));
  }

  bundle.options.request = &request;
  // Soft turn-angle penalty at every path vertex: suppresses the
  // near-180-degree reversals at SID/STAR handoff fixes. Always on for routing;
  // the calibration lives in the TurnPenalty constants (tunable there).
  bundle.options.turn_penalty.enabled = true;

  if (request.altitude.has_value()) {
    bundle.altitude_band = std::make_unique<AltitudeBandConstraint>();
    bundle.mora = std::make_unique<MoraConstraint>(mora_grid);
    bundle.options.constraints.push_back(bundle.altitude_band.get());
    bundle.options.constraints.push_back(bundle.mora.get());
  }
  if (request.level != LevelPreference::kNone) {
    bundle.level_pref = std::make_unique<LevelPreferenceConstraint>();
    bundle.options.constraints.push_back(bundle.level_pref.get());
  }
  if (!request.avoid_waypoints.empty()) {
    // Pass a copy: AvoidWaypointConstraint takes ownership of its sorted set,
    // while avoid_vertices must also remain available for endpoint pruning.
    bundle.avoid = std::make_unique<AvoidWaypointConstraint>(bundle.avoid_vertices);
    bundle.options.constraints.push_back(bundle.avoid.get());
  }
  if (bundle.airway_rules) {
    bundle.options.constraints.push_back(bundle.airway_rules.get());
  }
  if (request.random_seed.has_value()) {
    bundle.randomize = std::make_unique<RandomizeConstraint>(*request.random_seed);
    bundle.options.constraints.push_back(bundle.randomize.get());
  }

  // Airports must not be transit nodes: their synthetic DCT links would let the
  // search cut through an unrelated airport (e.g. ...MIE DCT KMIE SNKPT...).
  // Endpoints connect via seeded connection fixes, not airport vertices, so
  // blocking all airport vertices as intermediate nodes is safe. Airports occupy
  // the contiguous tail [first_airport_vertex, VertexCount), so a NodeFilter
  // range check replaces the old IsAirport std::function -- an inlined
  // two-compare on the hot loop instead of a type-erased call per neighbor.
  const NavGraph& graph = builder.graph();
  bundle.options.node_filter =
      NodeFilter{builder.first_airport_vertex(), graph.VertexCount(), nullptr};

  return Result<ConstraintBundle>::Ok(std::move(bundle));
}

}  // namespace bf
