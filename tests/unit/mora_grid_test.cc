// SPDX-License-Identifier: MIT
#include "core/domain/mora_grid.h"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

using bf::Coordinate;
using bf::MoraGrid;

TEST_CASE("MoraGrid: empty grid reports no floor", "[unit][mora_grid]") {
  MoraGrid g;
  CHECK(g.Empty());
  CHECK(g.MoraAt(Coordinate{40.0, -74.0}) == 0);
}

TEST_CASE("MoraGrid: SetCell then MoraAt within the cell", "[unit][mora_grid]") {
  MoraGrid g;
  g.SetCell(40, -75, 120);
  CHECK_FALSE(g.Empty());
  // Any coordinate whose floor is (40, -75) resolves to the same cell.
  CHECK(g.MoraAt(Coordinate{40.0, -75.0}) == 120);
  CHECK(g.MoraAt(Coordinate{40.7, -74.3}) == 120);  // floor(-74.3) == -75
  // A neighbouring cell is still unknown.
  CHECK(g.MoraAt(Coordinate{41.0, -75.0}) == 0);
}

TEST_CASE("MoraGrid: negative coordinates floor toward -inf", "[unit][mora_grid]") {
  MoraGrid g;
  g.SetCell(-1, -1, 55);
  CHECK(g.MoraAt(Coordinate{-0.5, -0.5}) == 55);  // floor(-0.5) == -1
  CHECK(g.MoraAt(Coordinate{0.5, 0.5}) == 0);     // cell (0,0), unset
}

TEST_CASE("MoraGrid: out-of-range indices are rejected", "[unit][mora_grid]") {
  MoraGrid g;
  // The grid is unwrapped: lat +90 and lon +180 have no row/column.
  g.SetCell(90, 0, 300);   // ignored (lat out of range)
  g.SetCell(0, 180, 300);  // ignored (lon out of range)
  CHECK(g.Empty());
  CHECK(g.MoraAt(Coordinate{90.0, 0.0}) == 0);
  CHECK(g.MoraAt(Coordinate{0.0, 180.0}) == 0);
  // The extreme valid corners do work.
  g.SetCell(89, 179, 210);
  g.SetCell(-90, -180, 220);
  CHECK(g.MoraAt(Coordinate{89.0, 179.0}) == 210);
  CHECK(g.MoraAt(Coordinate{-90.0, -180.0}) == 220);
}

TEST_CASE("MoraGrid: non-finite coordinates return 0 (no UB)", "[unit][mora_grid]") {
  MoraGrid g;
  g.SetCell(0, 0, 99);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  CHECK(g.MoraAt(Coordinate{nan, 0.0}) == 0);
  CHECK(g.MoraAt(Coordinate{0.0, inf}) == 0);
  CHECK(g.MoraAt(Coordinate{1.0e18, 0.0}) == 0);  // out of int range guard
}

TEST_CASE("MoraGrid: overwriting a cell keeps the populated count consistent",
          "[unit][mora_grid]") {
  MoraGrid g;
  g.SetCell(10, 20, 100);
  CHECK_FALSE(g.Empty());
  // Overwriting one populated cell with another non-zero value must not change
  // the count (still exactly one populated cell).
  g.SetCell(10, 20, 200);
  CHECK(g.MoraAt(Coordinate{10.0, 20.0}) == 200);
  CHECK_FALSE(g.Empty());
  // Clearing the only populated cell back to 0 must decrement the count so the
  // grid reports empty again (the invariant FromCells relies on).
  g.SetCell(10, 20, 0);
  CHECK(g.MoraAt(Coordinate{10.0, 20.0}) == 0);
  CHECK(g.Empty());
}

TEST_CASE("MoraGrid: FromCells rejects a wrong-sized array", "[unit][mora_grid]") {
  std::vector<int16_t> too_small(10, 5);
  const MoraGrid g = MoraGrid::FromCells(std::move(too_small));
  CHECK(g.Empty());  // bad size -> empty grid, no floor anywhere
  CHECK(g.MoraAt(Coordinate{0.0, 0.0}) == 0);
}

TEST_CASE("MoraGrid: FromCells accepts the exact size and recomputes count", "[unit][mora_grid]") {
  std::vector<int16_t> cells(static_cast<size_t>(MoraGrid::kLatCount) * MoraGrid::kLonCount, 0);
  // Populate the south-west corner cell (lat -90, lon -180) => index 0.
  cells[0] = 150;
  const MoraGrid g = MoraGrid::FromCells(std::move(cells));
  CHECK_FALSE(g.Empty());
  CHECK(g.MoraAt(Coordinate{-90.0, -180.0}) == 150);
  CHECK(g.MoraAt(Coordinate{0.0, 0.0}) == 0);
}
