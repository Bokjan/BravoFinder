// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "core/constraints/constraint.h"
#include "core/routing/route_request.h"

namespace bf {

// Whether `region` starts with any entry of `prefixes`. An empty `prefixes`, or
// any empty entry in it, matches everything -- that is how a rule spells "every
// region" (the CLI writes it "*"). Shared by the resolver and its tests so both
// agree on the matching rule.
//
// Prefix, not equality, per the rule semantics: "Z" names all Z-something FIRs
// while "ZB" names just one. A two-letter entry is effectively exact, since no
// longer region code exists to extend it -- which is why AirwayRule has no Match
// mode for the region side.
inline bool MatchesAnyPrefix(std::string_view region, const std::vector<std::string>& prefixes) {
  if (prefixes.empty()) {
    return true;
  }
  for (const std::string& p : prefixes) {
    if (p.empty() || region.starts_with(p)) {
      return true;
    }
  }
  return false;
}

// Whether `designator` matches any entry of `wanted` under `match`. An empty
// `wanted`, or any empty entry, matches everything. Unlike the region side this
// needs both modes: a category rule ("all J routes") must be a prefix, while a
// single-airway rule must be exact -- 1371 designators in AIRAC 2601 are a strict
// prefix of another one, so a prefix "J60" would also catch J603/J604/J605.
inline bool MatchesAnyDesignator(std::string_view designator,
                                 const std::vector<std::string>& wanted, AirwayRule::Match match) {
  if (wanted.empty()) {
    return true;
  }
  for (const std::string& w : wanted) {
    if (w.empty()) {
      return true;
    }
    if (match == AirwayRule::Match::kPrefix ? designator.starts_with(w) : designator == w) {
      return true;
    }
  }
  return false;
}

// Region + designator airway rules (see AirwayRule), as a routing constraint.
// Blocks or penalizes each airway leg whose designator matches a rule's
// designator set and whose either endpoint sits in one of the rule's regions.
//
// Like the avoid constraint, all name matching happens BEFORE the search: the
// caller resolves the rules into two bitmask tables (see ResolveAirwayRules in the
// routing layer, which owns the GraphBuilder dependency this header must not
// have). Bit i of both tables corresponds to rule i, so a leg matches rule i iff
// its region bit and its designator bit are both set -- and the hot path is two
// array reads plus bit operations, cheaper than a binary_search.
//
// Why per-leg rather than per-airway-name: see the AirwayRule comment. The short
// version is that a designator names several disjoint physical airways in real
// data, so a name-level ban leaks across regions.
//
// Rule count is capped at kMaxRules by the uint32_t mask width. That is far above
// any real use, and it caps RULES only: because both the region and the designator
// side are lists sharing one bit, even a long "block these 200 airways exactly"
// list is a single rule. The resolver rejects an over-long rule list rather than
// silently dropping rules.
class AirwayRuleConstraint : public Constraint {
 public:
  // Max rules one query may carry, set by the bit width of the mask tables. This
  // counts RULES, not regions or designators: a single rule may enumerate any
  // number of either, since all its entries share one bit.
  static constexpr size_t kMaxRules = 32;

  // Takes the resolved tables. `vertex_mask[v]` holds the bits of the rules whose
  // regions match vertex v; `airway_mask[id]` the bits of those whose prefix
  // matches airway id (entry 0, "DCT", is always 0 so synthetic edges are never
  // ruled on). `block_bits` has bit i set iff rule i is kBlock, and
  // `fractions[i]` is rule i's penalty fraction (0 for a block rule, which never
  // reaches the penalty sum).
  AirwayRuleConstraint(std::vector<uint32_t> vertex_mask, std::vector<uint32_t> airway_mask,
                       uint32_t block_bits, std::vector<double> fractions)
      : vertex_mask_(std::move(vertex_mask)),
        airway_mask_(std::move(airway_mask)),
        block_bits_(block_bits),
        fractions_(std::move(fractions)) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest&) const override {
    // A leg matches rule i iff EITHER endpoint's region matches (hence the OR of
    // the two vertex masks) AND the airway's designator matches.
    const uint32_t m =
        (vertex_mask_[ctx.from] | vertex_mask_[ctx.edge.to]) & airway_mask_[ctx.edge.airway_id];
    if (m == 0) {
      return EdgeVerdict::Allow();  // the overwhelmingly common case
    }
    // Any blocking rule wins outright, short-circuiting the penalty sum: a
    // blocked edge's extra_cost is meaningless (the search drops the edge on
    // !allowed without reading it).
    if ((m & block_bits_) != 0) {
      return EdgeVerdict::Block();
    }
    // Sum the fractions of every matching penalize rule. Summing, not taking the
    // max: each rule states an independent reason to avoid the leg ("this is a J
    // route in China" and "this is a V airway" both apply), and it mirrors how
    // the search already sums soft penalties across the constraint chain. The sum
    // may exceed 1.0, which is intended -- a user can stack rules toward
    // block-like strength while keeping the graph connected. Every fraction is
    // non-negative, so the heuristic stays a lower bound.
    double frac = 0.0;
    for (uint32_t bits = m; bits != 0; bits &= bits - 1) {
      frac += fractions_[static_cast<size_t>(std::countr_zero(bits))];
    }
    return EdgeVerdict::Penalize(ctx.edge.distance_nm * frac);
  }

 private:
  std::vector<uint32_t> vertex_mask_;  // per-vertex: rules whose region matches. size = V
  std::vector<uint32_t> airway_mask_;  // per-airway_id: rules whose prefix matches. size = names
  uint32_t block_bits_ = 0;            // bit i set iff rule i blocks
  std::vector<double> fractions_;      // per-rule penalty fraction; size = rule count
};

}  // namespace bf
