// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/constraints/constraint.h"

namespace bf {

// Hard filter: forbid routing through a set of waypoints and/or over a set of
// airways. A vertex is avoided by blocking every edge that enters it (so it
// never appears as an intermediate point); the route's own endpoints are
// unaffected since they are seeded, not entered via an edge. An airway is
// avoided by blocking every edge whose airway_id is in the set.
//
// The airway_id set is resolved at construction from user-supplied designators:
// because concurrent airways are stored as a single combined name ("A593-Y592"),
// the caller expands each combined name to its designators and includes the
// airway_id whenever any designator matches. This constraint therefore only
// does lookups on the hot path -- no string work per edge.
//
// The hot path evaluates this constraint on every edge of every search (and
// every Yen spur), so the lookups must be cheaper than a hash-set probe. Both
// avoid sets are small in practice -- a handful of user-named waypoints and
// airways -- so each is kept as a sorted vector with binary_search: no hashing,
// no per-query large allocation, and the whole set fits in one cache line for
// the small N that actually occurs. A fixed-size bitset was considered for the
// airway set, but it would either hardcode the airway_id space or carry a large
// zero-fill per query for what is usually a 2-3 element set.
//
// Sortedness invariant: the members are const, initialized once from the
// caller's unordered_sets via MakeSorted (copy + sort). Because they are const,
// no later code can write them, so the vectors stay sorted for every
// binary_search in Evaluate -- the invariant is enforced by the type system,
// not by discipline.
class AvoidConstraint : public Constraint {
 public:
  AvoidConstraint(std::unordered_set<int> avoid_vertices,
                  std::unordered_set<uint16_t> avoid_airway_ids)
      : vertices_(MakeSorted(std::move(avoid_vertices))),
        airways_(MakeSorted(std::move(avoid_airway_ids))) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest&) const override {
    if (std::binary_search(vertices_.begin(), vertices_.end(), ctx.edge.to)) {
      return EdgeVerdict::Block();  // never enter an avoided waypoint
    }
    if (std::binary_search(airways_.begin(), airways_.end(), ctx.edge.airway_id)) {
      return EdgeVerdict::Block();  // never traverse an avoided airway segment
    }
    return EdgeVerdict::Allow();
  }

 private:
  // Copy an unordered_set into a vector and sort it once. The members below are
  // const, so this is the only code that ever writes them.
  template <typename T>
  static std::vector<T> MakeSorted(std::unordered_set<T> set) {
    std::vector<T> v(set.begin(), set.end());
    std::sort(v.begin(), v.end());
    return v;
  }

  const std::vector<int> vertices_;      // sorted, for binary_search
  const std::vector<uint16_t> airways_;  // sorted, expanded from designators
};

}  // namespace bf
