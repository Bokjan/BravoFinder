// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "core/domain/coordinate.h"
#include "core/graph/nav_graph.h"
#include "core/routing/route_request.h"

namespace bf {

// The verdict a constraint returns for one edge under a given request.
struct EdgeVerdict {
  bool allowed = true;      // false => hard filter: the edge cannot be used
  double extra_cost = 0.0;  // soft penalty added to the edge's traversal cost

  static EdgeVerdict Allow() { return {true, 0.0}; }
  static EdgeVerdict Block() { return {false, 0.0}; }
  static EdgeVerdict Penalize(double cost) { return {true, cost}; }
};

// Context for evaluating one directed edge: the edge itself and the coordinates
// of both endpoints, so position-dependent constraints (e.g. MORA, which must
// sample terrain along the whole leg) can look up the relevant cells.
//
// All members are held by value. EdgeContext is a small aggregate of three
// 16-byte PODs -- the edge plus the two endpoint Coordinates -- built per edge
// on A*'s relaxation hot path; copying them is register-level, sub-nanosecond
// work. A reference member would trade that copy for an indirection on every
// field read inside a constraint (MORA reads both endpoints several times, and
// the altitude constraints read the edge), with no measured benefit. Holding
// everything by value keeps EdgeContext free of lifetime traps: it can be bound
// to temporary edge/Coordinate values (as the constraint tests do) without
// dangling, and it never depends on any referent outliving the context.
struct EdgeContext {
  GraphEdge edge;
  Coordinate from_coord;
  Coordinate to_coord;
};

// A pluggable routing constraint. Each constraint inspects an edge under the
// active request and returns whether the edge is usable (hard filter) and/or an
// extra traversal cost (soft penalty). Constraints are stateless and combined
// by the search: an edge is blocked if any constraint blocks it; its penalties
// sum.
class Constraint {
 public:
  virtual ~Constraint() = default;
  virtual EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const = 0;
};

}  // namespace bf
