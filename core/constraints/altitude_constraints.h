#pragma once

#include "core/constraints/constraint.h"

namespace bf {

// Hard filter: when a cruise altitude is given, an airway segment is usable
// only if that flight level falls within the segment's [base_fl, top_fl] band.
// Segments with no altitude limits recorded (base_fl == 0 && top_fl == 0, e.g.
// synthetic DCT edges) are exempt. With no cruise altitude set, this constraint
// allows everything.
class AltitudeBandConstraint : public Constraint {
 public:
  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const override {
    if (!request.cruise_fl.has_value()) {
      return EdgeVerdict::Allow();
    }
    const GraphEdge& e = ctx.edge;
    if (e.base_fl == 0 && e.top_fl == 0) {
      return EdgeVerdict::Allow();  // no recorded band (e.g. DCT)
    }
    const int fl = *request.cruise_fl;
    if (fl < e.base_fl || fl > e.top_fl) {
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
    const bool wants_high = request.level == LevelPreference::kHigh;
    if (EdgeIsHigh(ctx.edge) == wants_high) {
      return EdgeVerdict::Allow();
    }
    return EdgeVerdict::Penalize(ctx.edge.distance_nm * penalty_fraction_);
  }

 private:
  double penalty_fraction_;
};

}  // namespace bf
