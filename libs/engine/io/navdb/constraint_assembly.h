// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <memory>
#include <vector>

#include "core/constraints/airway_rule_constraint.h"
#include "core/constraints/altitude_constraints.h"
#include "core/constraints/avoid_waypoint_constraint.h"
#include "core/constraints/mora_constraint.h"
#include "core/constraints/randomize_constraint.h"
#include "core/graph/astar.h"
#include "core/result.h"

namespace bf {

class GraphBuilder;
class MoraGrid;
struct RouteRequest;

// Owns the constraint objects whose addresses live in SearchOptions::constraints
// for the duration of a FindRoutes search. Heap-backed unique_ptrs keep those
// pointers valid across moves of this bundle (heap addresses do not change).
//
// Lifetime (do not stash across callers):
// - `options.request` points at the RouteRequest passed to Build — that request
//   must outlive every use of this bundle (including the search).
// - `mora` (when non-null) holds a const reference into the MoraGrid passed to
//   Build — that grid must outlive the bundle (typically NavDatabase::mora_).
// FindRoutes keeps the bundle on the stack for the search only; do not return
// or cache a ConstraintBundle beyond those two referents.
struct ConstraintBundle {
  SearchOptions options;
  // Sorted, deduped avoid set — also used to prune seeded endpoints and reject
  // forced points that collide with avoid (AvoidWaypointConstraint only blocks
  // edges entering a vertex, not seeded starts/ends).
  std::vector<int> avoid_vertices;

  // Owned storage; only the constraints pushed into options.constraints are
  // non-null. Kept as unique_ptr so a returned Bundle may be moved without
  // invalidating the raw pointers in options.constraints.
  std::unique_ptr<AltitudeBandConstraint> altitude_band;
  std::unique_ptr<MoraConstraint> mora;
  std::unique_ptr<LevelPreferenceConstraint> level_pref;
  std::unique_ptr<AvoidWaypointConstraint> avoid;
  std::unique_ptr<AirwayRuleConstraint> airway_rules;
  std::unique_ptr<RandomizeConstraint> randomize;
};

// Builds SearchOptions (+ owned constraint storage) from a RouteRequest.
// Stateless; all entry points are static.
class ConstraintAssembly {
 public:
  ConstraintAssembly() = delete;

  // Assemble altitude / MORA / level / avoid / airway_rules / randomize /
  // turn_penalty / airport node_filter. Returns Err when airway_rules exceeds
  // AirwayRuleConstraint::kMaxRules. See ConstraintBundle lifetime notes:
  // `request` and `mora` must outlive the returned bundle.
  static Result<ConstraintBundle> Build(const RouteRequest& request, const GraphBuilder& builder,
                                        const MoraGrid& mora);
};

}  // namespace bf
