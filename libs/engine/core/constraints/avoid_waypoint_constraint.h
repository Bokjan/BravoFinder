// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "core/constraints/constraint.h"

namespace bf {

// Hard filter: forbid routing through a set of waypoints. A vertex is avoided by
// blocking every edge that enters it (so it never appears as an intermediate
// point); the route's own endpoints are unaffected since they are seeded, not
// entered via an edge -- the routing layer prunes an avoided fix from the seeded
// endpoint sets separately.
//
// Airway-level bans live in AirwayRuleConstraint, not here: "avoid J60" is a rule
// with designators {"J60"}, match kExact and action kBlock (which additionally
// supports confining the ban to given regions). Keeping one code path for airway
// restrictions means "forbid this airway" has one spelling and one implementation.
//
// The hot path evaluates this constraint on every edge of every search (and every
// Yen spur), so the lookup must be cheaper than a hash-set probe. The avoid set is
// small in practice -- a handful of user-named waypoints -- so it is kept as a
// sorted vector with binary_search: no hashing, no per-query large allocation, and
// the whole set fits in one cache line for the small N that actually occurs.
//
// Sortedness invariant: the member is const, initialized once via DedupSort (sort
// + unique) from the caller's vector. Because it is const, no later code can write
// it, so the vector stays sorted for every binary_search in Evaluate -- the
// invariant is enforced by the type system, not by discipline.
class AvoidWaypointConstraint : public Constraint {
 public:
  // Takes the avoid set as a vector. ResolveAvoidVertices produces this straight
  // from the request, so the unordered_set hashing and the set->vector conversion
  // the old interface paid for are avoided. DedupSort re-sorts idempotently,
  // keeping the member sorted regardless of the exact caller input.
  explicit AvoidWaypointConstraint(std::vector<int> avoid_vertices)
      : vertices_(DedupSort(std::move(avoid_vertices))) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest&) const override {
    if (std::binary_search(vertices_.begin(), vertices_.end(), ctx.edge.to)) {
      return EdgeVerdict::Block();  // never enter an avoided waypoint
    }
    return EdgeVerdict::Allow();
  }

 private:
  // Sort and dedup a vector (idempotent if the caller already did so) so the
  // member is guaranteed sorted and unique regardless of input, keeping
  // binary_search correct in Evaluate.
  static std::vector<int> DedupSort(std::vector<int> v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
  }

  const std::vector<int> vertices_;  // sorted, for binary_search
};

}  // namespace bf
