#pragma once

#include <algorithm>
#include <cmath>

#include "core/constraints/constraint.h"
#include "core/domain/mora_grid.h"

namespace bf {

// Hard filter: when a cruise altitude range is given, an edge is usable only if
// some level in that range is at or above the grid minimum off-route altitude
// (MORA) along the whole leg -- i.e. the range's top clears the floor
// everywhere on the way. This keeps routes able to hold a safe height over
// terrain/obstructions. Cells with no MORA data (value 0) impose no limit.
//
// Sampling: MORA is a 1-degree grid, so a long leg may cross several cells,
// including a high-MORA cell that neither endpoint sits in. The constraint
// samples the leg's great-circle track (linearly interpolated in lat/lon, which
// is accurate enough at a 1-degree grid resolution) and takes the maximum MORA
// across the sampled points plus both endpoints. Short legs collapse to just
// the endpoints, matching the previous behavior.
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
    const int16_t mora = MaxMoraAlongLeg(ctx.from_coord, ctx.to_coord);
    if (mora == 0) {
      return EdgeVerdict::Allow();  // no cell along the leg has a known floor
    }
    if (request.altitude->max_fl < mora) {
      return EdgeVerdict::Block();  // even the top of the range is below MORA
    }
    return EdgeVerdict::Allow();
  }

 private:
  // The highest known MORA on the leg, sampling the track at ~0.5-degree
  // spacing (finer than the 1-degree grid, so no cell is skipped) and including
  // both endpoints. Returns 0 when every sampled cell is unknown.
  int16_t MaxMoraAlongLeg(const Coordinate& from, const Coordinate& to) const {
    int16_t best = 0;
    best = std::max(best, grid_.MoraAt(from));
    best = std::max(best, grid_.MoraAt(to));
    // Sample the straight-line lat/lon interpolation; for a 1-degree grid this
    // is close enough to the great-circle track that no cell is missed.
    const double dlat = to.latitude - from.latitude;
    const double dlon = to.longitude - from.longitude;
    // Degrees of track spanned; sample roughly every 0.5 degrees.
    const double span = std::max(std::abs(dlat), std::abs(dlon));
    const int steps = std::max(1, static_cast<int>(std::ceil(span / 0.5)));
    for (int i = 1; i < steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      best = std::max(
          best, grid_.MoraAt(Coordinate{from.latitude + dlat * t, from.longitude + dlon * t}));
    }
    return best;
  }

  const MoraGrid& grid_;
};

}  // namespace bf
