#include <catch2/catch_test_macros.hpp>

#include "core/constraints/altitude_constraints.h"
#include "core/constraints/mora_constraint.h"
#include "core/domain/mora_grid.h"

namespace {

bf::GraphEdge MakeEdge(int base_fl, int top_fl, bool is_high) {
  bf::GraphEdge e;
  e.to = 1;
  e.distance_nm = 100.0;
  e.airway_id = 1;
  e.base_fl = static_cast<int16_t>(base_fl);
  e.top_fl = static_cast<int16_t>(top_fl);
  e.is_high = is_high;
  return e;
}

bf::RouteRequest WithAltitude(int fl) {
  bf::RouteRequest r;
  r.cruise_fl = fl;
  return r;
}

TEST_CASE("altitude band: no cruise altitude allows everything", "[constraint]") {
  bf::AltitudeBandConstraint c;
  bf::RouteRequest r;  // no cruise_fl
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, r).allowed);
}

TEST_CASE("altitude band: blocks outside the band", "[constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitude(350)).allowed);        // inside
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(100)).allowed);  // below base
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(500)).allowed);  // above top
}

TEST_CASE("altitude band: zero band (DCT) is exempt", "[constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitude(350)).allowed);
}

TEST_CASE("level preference: penalizes non-preferred level", "[constraint]") {
  bf::LevelPreferenceConstraint c(0.5);
  bf::RouteRequest r;
  r.level = bf::LevelPreference::kHigh;

  bf::EdgeContext high{MakeEdge(180, 450, true), bf::Coordinate{}};
  bf::EdgeContext low{MakeEdge(0, 180, false), bf::Coordinate{}};
  CHECK(c.Evaluate(high, r).extra_cost == 0.0);  // preferred: no penalty
  bf::EdgeVerdict v = c.Evaluate(low, r);
  CHECK(v.allowed);             // not forbidden
  CHECK(v.extra_cost == 50.0);  // 100 NM * 0.5
}

TEST_CASE("MORA: blocks below grid minimum, allows at or above", "[constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(40, -74, 100);  // cell covering ~JFK area: MORA FL100
  bf::MoraConstraint c(grid);

  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{40.5, -73.5}};
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(80)).allowed);  // below MORA
  CHECK(c.Evaluate(ctx, WithAltitude(100)).allowed);       // at MORA
  CHECK(c.Evaluate(ctx, WithAltitude(150)).allowed);       // above MORA
}

TEST_CASE("MORA: unknown cell imposes no limit", "[constraint]") {
  bf::MoraGrid grid;  // empty
  bf::MoraConstraint c(grid);
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{10.0, 20.0}};
  CHECK(c.Evaluate(ctx, WithAltitude(50)).allowed);
}

TEST_CASE("MORA grid floors negative coordinates correctly", "[constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(-1, -1, 50);
  // A point at (-0.5, -0.5) floors to cell (-1, -1).
  CHECK(grid.MoraAt(bf::Coordinate{-0.5, -0.5}) == 50);
  // A point at (0.5, 0.5) floors to cell (0, 0): unset.
  CHECK(grid.MoraAt(bf::Coordinate{0.5, 0.5}) == 0);
}

}  // namespace
