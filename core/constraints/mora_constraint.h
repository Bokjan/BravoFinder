#pragma once

#include "core/constraints/constraint.h"
#include "core/domain/mora_grid.h"

namespace bf {

// Hard filter: when a cruise altitude is given, an edge is usable only if that
// flight level is at or above the grid minimum off-route altitude (MORA) of the
// cell the edge enters. This keeps routes above terrain/obstruction safe
// heights. Cells with no MORA data (value 0) impose no limit.
//
// Note: MORA is an MSL altitude while a flight level is a pressure altitude;
// comparing them directly is a reasonable safety-floor approximation for now.
class MoraConstraint : public Constraint {
 public:
  explicit MoraConstraint(const MoraGrid& grid) : grid_(grid) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const override {
    if (!request.cruise_fl.has_value()) {
      return EdgeVerdict::Allow();
    }
    const int16_t mora = grid_.MoraAt(ctx.to_coord);
    if (mora == 0) {
      return EdgeVerdict::Allow();  // unknown cell, no lower bound
    }
    if (*request.cruise_fl < mora) {
      return EdgeVerdict::Block();
    }
    return EdgeVerdict::Allow();
  }

 private:
  const MoraGrid& grid_;
};

}  // namespace bf
