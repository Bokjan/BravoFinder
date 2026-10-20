// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "core/constraints/constraint.h"
#include "core/domain/coordinate.h"
#include "core/domain/mora_grid.h"

namespace bf {

// Great-circle piece length for MaxMoraAlongLeg, in degrees of arc. A leg
// longer than this is split along its great circle first, so the lat/lon chord
// of each piece stays within a few hundredths of a degree of the true track
// even at high latitude, where a long east-west great circle bulges poleward.
inline constexpr double kMoraGreatCirclePieceDeg = 5.0;

// Hard filter: when a cruise altitude range is given, an edge is usable only if
// some level in that range is at or above the grid minimum off-route altitude
// (MORA) along the whole leg -- i.e. the range's top clears the floor
// everywhere on the way. This keeps routes able to hold a safe height over
// terrain/obstructions. Cells with no MORA data (value 0) impose no limit.
//
// Track coverage: MORA is a 1-degree grid, so a long leg may cross several
// cells, including a high-MORA cell that neither endpoint sits in. The
// constraint takes the maximum over every cell the leg's track passes through:
// the track is split along its great circle into pieces of at most
// kMoraGreatCirclePieceDeg, and each piece's lat/lon chord is walked cell by
// cell (a grid traversal, not point sampling, so a cell the track only clips
// at a corner still counts).
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
  // The highest known MORA in any cell the leg passes through. Returns 0 when
  // every such cell is unknown.
  int16_t MaxMoraAlongLeg(const Coordinate& from, const Coordinate& to) const {
    // Guard against non-finite coordinates (a corrupted graph from a bad parse):
    // the traversal's cell counts would be NaN, and casting NaN to int is UB.
    // Bail to "unknown" so the leg is allowed, matching how MoraAt treats a bad
    // index.
    if (!std::isfinite(from.latitude) || !std::isfinite(from.longitude) ||
        !std::isfinite(to.latitude) || !std::isfinite(to.longitude)) {
      return 0;
    }
    // Regularize the longitude delta to the shorter way around the globe: a leg
    // from +179 to -179 spans 2 degrees, not 358, and must not read the strip
    // of cells the long way round.
    const double dlon = std::remainder(to.longitude - from.longitude, kDegreesFullCircle);
    int16_t best = 0;
    // The common case -- an airway leg spanning a few degrees -- walks its
    // lat/lon chord directly, with no trigonometry on the search's hot path.
    if (std::max(std::abs(to.latitude - from.latitude), std::abs(dlon)) <=
        kMoraGreatCirclePieceDeg) {
      WalkCells(from.latitude, from.longitude, to.latitude, from.longitude + dlon, best);
      return best;
    }
    constexpr double kDegToRad = kPi / kDegreesHalfCircle;
    const double arc_deg = from.DistanceTo(to) / kEarthRadiusNm / kDegToRad;
    const int pieces = std::max(2, static_cast<int>(std::ceil(arc_deg / kMoraGreatCirclePieceDeg)));
    // Points along the great circle by spherical linear interpolation between
    // the endpoints' unit vectors. Each piece starts where the last ended, with
    // its longitude unwrapped relative to it so a piece never wraps.
    const auto unit = [kDegToRad](const Coordinate& c) {
      const double lat = c.latitude * kDegToRad;
      const double lon = c.longitude * kDegToRad;
      return std::array<double, 3>{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon),
                                   std::sin(lat)};
    };
    const std::array<double, 3> a = unit(from);
    const std::array<double, 3> b = unit(to);
    const double omega = arc_deg * kDegToRad;
    const double sin_omega = std::sin(omega);
    if (!(sin_omega > 1e-9)) {
      // A degenerate arc: a wide longitude span at the pole (a near-zero arc) or
      // an antipodal pair, neither an airway. Fall back to the chord rather than
      // divide by ~0.
      WalkCells(from.latitude, from.longitude, to.latitude, from.longitude + dlon, best);
      return best;
    }
    double prev_lat = from.latitude;
    double prev_lon = from.longitude;
    for (int i = 1; i <= pieces; ++i) {
      double lat = to.latitude;
      double lon = to.longitude;
      if (i < pieces) {
        const double t = static_cast<double>(i) / pieces;
        const double wa = std::sin((1.0 - t) * omega) / sin_omega;
        const double wb = std::sin(t * omega) / sin_omega;
        const double x = wa * a[0] + wb * b[0];
        const double y = wa * a[1] + wb * b[1];
        const double z = wa * a[2] + wb * b[2];
        lat = std::atan2(z, std::hypot(x, y)) / kDegToRad;
        lon = std::atan2(y, x) / kDegToRad;
      }
      const double next_lon = prev_lon + std::remainder(lon - prev_lon, kDegreesFullCircle);
      WalkCells(prev_lat, prev_lon, lat, next_lon, best);
      prev_lat = lat;
      prev_lon = next_lon;
    }
    return best;
  }

  // Fold into `best` the MORA of every grid cell the straight lat/lon segment
  // (lat0, lon0) -> (lat1, lon1) passes through, endpoints' cells included.
  // Longitudes may run past +/-180 (unwrapped); MoraAtCell wraps the column.
  // A grid traversal: step to whichever cell boundary the segment reaches
  // next, so the visit count is exactly the number of boundaries crossed.
  void WalkCells(double lat0, double lon0, double lat1, double lon1, int16_t& best) const {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    int row = static_cast<int>(std::floor(lat0));
    int col = static_cast<int>(std::floor(lon0));
    const int end_row = static_cast<int>(std::floor(lat1));
    const int end_col = static_cast<int>(std::floor(lon1));
    best = std::max(best, grid_.MoraAtCell(row, col));
    const double dlat = lat1 - lat0;
    const double dlon = lon1 - lon0;
    const int step_row = dlat > 0.0 ? 1 : -1;
    const int step_col = dlon > 0.0 ? 1 : -1;
    // Parameter t in [0, 1] at which the segment next crosses a row / column
    // boundary, and how much t advances per further boundary.
    double next_row_t = kInf;
    double next_col_t = kInf;
    double row_t_step = kInf;
    double col_t_step = kInf;
    if (dlat != 0.0) {
      next_row_t = ((dlat > 0.0 ? row + 1.0 : static_cast<double>(row)) - lat0) / dlat;
      row_t_step = 1.0 / std::abs(dlat);
    }
    if (dlon != 0.0) {
      next_col_t = ((dlon > 0.0 ? col + 1.0 : static_cast<double>(col)) - lon0) / dlon;
      col_t_step = 1.0 / std::abs(dlon);
    }
    // The boundary count is fixed up front, so rounding in the t values can
    // only change which neighbour is visited at a near-corner, never loop.
    const int crossings = std::abs(end_row - row) + std::abs(end_col - col);
    for (int i = 0; i < crossings; ++i) {
      const bool row_first = row != end_row && (col == end_col || next_row_t < next_col_t);
      if (row_first) {
        row += step_row;
        next_row_t += row_t_step;
      } else {
        col += step_col;
        next_col_t += col_t_step;
      }
      best = std::max(best, grid_.MoraAtCell(row, col));
    }
  }

  const MoraGrid& grid_;
};

}  // namespace bf
