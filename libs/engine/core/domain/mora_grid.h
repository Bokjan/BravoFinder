// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "core/domain/coordinate.h"

namespace bf {

// Grid Minimum Off-Route Altitude: a 1-degree global grid of minimum safe
// altitudes (in flight levels, hundreds of feet). Each cell covers one degree
// of latitude and longitude, keyed by the integer floor of (lat, lon).
//
// Built by the loader via SetCell; queried with MoraAt. A value of 0 means
// "unknown" (no data for that cell), which callers treat as no lower bound.
class MoraGrid {
 public:
  static constexpr int kLatCount = 180;  // -90 .. +89
  static constexpr int kLonCount = 360;  // -180 .. +179
  // FloorToInt sentinel for non-finite / out-of-int input (Index rejects idx < 0).
  static constexpr int kInvalidGridIndex = -9999;
  // Reject coordinates that would overflow int when cast; ~1e9 stays in range.
  static constexpr double kFiniteValueBound = 1.0e9;

  MoraGrid() : cells_(kLatCount * kLonCount, 0) {}

  // Set the MORA (flight level) for the cell whose south-west corner is the
  // integer (lat, lon). Out-of-range indices are ignored.
  void SetCell(int lat, int lon, int16_t mora_fl) {
    const int idx = Index(lat, lon);
    if (idx < 0) {
      return;
    }
    // Keep populated_ in step with the non-zero cell count (the invariant
    // FromCells relies on, and Empty() reads): a first population increments,
    // and clearing a populated cell back to 0 decrements. No current loader
    // writes 0 (they guard with value > 0), but SetCell honors the contract
    // regardless.
    const bool was_populated = cells_[idx] != 0;
    const bool now_populated = mora_fl != 0;
    if (!was_populated && now_populated) {
      ++populated_;
    } else if (was_populated && !now_populated) {
      --populated_;
    }
    cells_[idx] = mora_fl;
  }

  // The MORA flight level at a position, or 0 if unknown / out of range.
  // The antimeridian (+180) maps onto the -180 column and the north pole (+90)
  // onto the +89 row (those exact grid seams have no own cell), so safety
  // constraints do not silently treat the boundary as unknown. Only the exact
  // boundary is remapped; coordinates beyond it (e.g. a corrupt 180.5) stay out
  // of range and return 0.
  int16_t MoraAt(const Coordinate& c) const {
    int lat = FloorToInt(c.latitude);
    int lon = FloorToInt(c.longitude);
    if (c.latitude == 90.0) {
      lat = 89;
    }
    if (c.longitude == 180.0) {
      lon = -180;
    }
    const int idx = Index(lat, lon);
    return idx >= 0 ? cells_[idx] : 0;
  }

  bool Empty() const { return populated_ == 0; }

  // The raw cell array (row-major, size kLatCount * kLonCount) for
  // serialization. Values are flight levels; 0 means unknown.
  const std::vector<int16_t>& cells() const { return cells_; }

  // Rebuild a grid from a serialized cell array. `cells` must have exactly
  // kLatCount * kLonCount entries; otherwise an empty grid is returned. The
  // populated count is recomputed from the non-zero cells.
  static MoraGrid FromCells(std::vector<int16_t> cells) {
    MoraGrid g;
    if (cells.size() != static_cast<size_t>(kLatCount) * kLonCount) {
      return g;
    }
    g.cells_ = std::move(cells);
    for (int16_t v : g.cells_) {
      if (v != 0) {
        ++g.populated_;
      }
    }
    return g;
  }

 private:
  static int FloorToInt(double v) {
    // Guard against non-finite or out-of-int-range input (a corrupted
    // coordinate from a bad parse): static_cast<int> of such a value is UB, so
    // bail to a sentinel that Index() then rejects as out of range.
    if (!std::isfinite(v) || v < -kFiniteValueBound || v > kFiniteValueBound) {
      return kInvalidGridIndex;
    }
    int i = static_cast<int>(v);
    if (v < 0 && static_cast<double>(i) != v) {
      --i;  // floor toward negative infinity
    }
    return i;
  }

  int Index(int lat, int lon) const {
    // After MoraAt's +180/+90 normalization, lon is in [-180, 179] and lat in
    // [-90, 89] for in-range coordinates. Values outside that (corrupt coords)
    // still return unknown.
    if (lat < -90 || lat > 89 || lon < -180 || lon > 179) {
      return -1;
    }
    return (lat + 90) * kLonCount + (lon + 180);
  }

  std::vector<int16_t> cells_;
  int populated_ = 0;
};

}  // namespace bf
