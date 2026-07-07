#pragma once

#include <cstdint>
#include <unordered_set>
#include <utility>

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
// does O(1) set lookups on the hot path -- no string work per edge.
class AvoidConstraint : public Constraint {
 public:
  AvoidConstraint(std::unordered_set<int> avoid_vertices,
                  std::unordered_set<uint16_t> avoid_airway_ids)
      : vertices_(std::move(avoid_vertices)), airways_(std::move(avoid_airway_ids)) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest&) const override {
    if (vertices_.count(ctx.edge.to) != 0) {
      return EdgeVerdict::Block();  // never enter an avoided waypoint
    }
    if (airways_.count(ctx.edge.airway_id) != 0) {
      return EdgeVerdict::Block();  // never traverse an avoided airway segment
    }
    return EdgeVerdict::Allow();
  }

 private:
  std::unordered_set<int> vertices_;
  std::unordered_set<uint16_t> airways_;  // already expanded from designators
};

}  // namespace bf
