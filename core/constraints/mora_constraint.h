#pragma once

#include "core/constraints/constraint.h"
#include "core/domain/mora_grid.h"

namespace bf {

// Hard filter: when a cruise altitude range is given, an edge is usable only if
// some level in that range is at or above the grid minimum off-route altitude
// (MORA) of the cell the edge enters -- i.e. the range's top clears the floor.
// This keeps routes able to hold a safe height over terrain/obstructions. Cells
// with no MORA data (value 0) impose no limit.
//
// Note: MORA is an MSL altitude while a flight level is a pressure altitude;
// comparing them directly is a reasonable safety-floor approximation for now.
class MoraConstraint : public Constraint {
 public:
  explicit MoraConstraint(const MoraGrid& grid) : grid_(grid) {}

  EdgeVerdict Evaluate(const EdgeContext& ctx, const RouteRequest& request) const override {
    if (!request.altitude.has_value()) {
      return EdgeVerdict::Allow();
    }
    const int16_t mora = grid_.MoraAt(ctx.to_coord);
    if (mora == 0) {
      return EdgeVerdict::Allow();  // unknown cell, no lower bound
    }
    if (request.altitude->max_fl < mora) {
      return EdgeVerdict::Block();  // even the top of the range is below MORA
    }
    return EdgeVerdict::Allow();
  }

 private:
  const MoraGrid& grid_;
};

}  // namespace bf
