// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "core/constraints/constraint.h"

namespace bf {

// Hard filter: when a cruise altitude range is given, an airway segment is
// usable only if that range overlaps the segment's [base_fl, top_fl] band. A
// bound recorded as 0 means "no limit on that side": base_fl == 0 is an open
// floor and top_fl == 0 is an open ceiling (e.g. synthetic DCT edges have both
// open; a high-altitude airway may have a floor but no published ceiling). With
// no cruise altitude set, this constraint allows everything.
class AltitudeBandConstraint : public Constraint {
 public:
  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const override {
    if (!request.altitude.has_value()) {
      return EdgeVerdict::Allow();
    }
    const GraphEdge& e = ctx.edge;
    const FlRange& r = *request.altitude;
    // Two inclusive ranges are disjoint iff one lies entirely below the other,
    // but only test a bound that is actually recorded: a 0 bound is open on that
    // side and constrains nothing. Testing an open (0) top_fl as if it were a
    // real ceiling would wrongly Block a valid high-altitude segment whose range
    // starts above FL0.
    if (e.base_fl != 0 && r.max_fl < e.base_fl) {
      return EdgeVerdict::Block();
    }
    if (e.top_fl != 0 && r.min_fl > e.top_fl) {
      return EdgeVerdict::Block();
    }
    return EdgeVerdict::Allow();
  }
};

// Soft penalty: nudges the search toward the preferred airway level (high/low)
// without forbidding the other. Edges not matching the preference get a
// distance-proportional penalty so a mildly longer preferred-level route is
// chosen over a shorter mixed one, but a non-preferred edge is still usable
// when no preferred alternative exists.
class LevelPreferenceConstraint : public Constraint {
 public:
  explicit LevelPreferenceConstraint(double penalty_fraction = 0.5)
      : penalty_fraction_(penalty_fraction) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const override {
    if (request.level == LevelPreference::kNone) {
      return EdgeVerdict::Allow();
    }
    // A both-level segment (DFD flightlevel 'B') is usable at either level, so it
    // is never penalized regardless of the preference.
    if (ctx.edge.level == AirwayLevel::kBoth) {
      return EdgeVerdict::Allow();
    }
    const bool wants_high = request.level == LevelPreference::kHigh;
    if ((ctx.edge.level == AirwayLevel::kHigh) == wants_high) {
      return EdgeVerdict::Allow();
    }
    return EdgeVerdict::Penalize(ctx.edge.distance_nm * penalty_fraction_);
  }

 private:
  double penalty_fraction_;
};

}  // namespace bf
