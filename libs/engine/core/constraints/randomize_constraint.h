// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>

#include "core/constraints/constraint.h"

namespace bf {

// Soft penalty: adds a small, deterministic, non-negative perturbation to each
// edge's cost, derived from a caller-supplied seed. This diversifies routes
// ("random routing") without breaking any invariant:
//
//   * Determinism: the perturbation is a pure function of (seed, edge), so the
//     same seed always yields the same route -- reproducible and testable, and
//     with no mutable/shared state it is safe under the const concurrency
//     contract (each search reads its own seed).
//   * Admissibility: the penalty is non-negative and the geometric heuristic is
//     unchanged, so it stays a lower bound on the perturbed cost -- A* remains
//     optimal (under the perturbed cost) and does not degrade toward Dijkstra.
//
// The perturbation is a fraction (eps) of the edge's own length scaled by a
// hash-derived value in [0, 1), so longer edges can be nudged more in absolute
// terms while the relative jitter stays bounded by eps.
class RandomizeConstraint : public Constraint {
 public:
  explicit RandomizeConstraint(uint32_t seed, double eps = 0.05) : seed_(seed), eps_(eps) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest&) const override {
    const double jitter = Hash01(seed_, ctx.edge.to, ctx.edge.airway_id);
    return EdgeVerdict::Penalize(ctx.edge.distance_nm * eps_ * jitter);
  }

 private:
  // Map (seed, to, airway_id) to a value in [0, 1) via a splitmix64-style
  // finalizer. Pure and stable across platforms (fixed-width integer math only).
  static double Hash01(uint32_t seed, int32_t to, uint16_t airway_id) {
    // Pack seed and `to` into the two non-overlapping halves of a 64-bit word,
    // then fold airway_id in through a separate mixing round. An earlier layout
    // XOR-ed seed<<32, to<<16 and airway_id, whose bits 32-47 overlapped, so
    // distinct (seed, to) pairs could cancel to the same value and collapse
    // route diversity (correctness was unaffected -- the jitter stays
    // non-negative and admissible either way).
    uint64_t x =
        (static_cast<uint64_t>(seed) << 32) | static_cast<uint64_t>(static_cast<uint32_t>(to));
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x ^= static_cast<uint64_t>(airway_id) + 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    x = x ^ (x >> 31);
    // Take the top 53 bits for a uniform double in [0, 1).
    return static_cast<double>(x >> 11) * (1.0 / 9007199254740992.0);  // 2^53
  }

  uint32_t seed_;
  double eps_;
};

}  // namespace bf
