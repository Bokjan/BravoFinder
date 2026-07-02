#include <catch2/catch_test_macros.hpp>

#include "core/constraints/altitude_constraints.h"
#include "core/constraints/avoid_constraint.h"
#include "core/constraints/mora_constraint.h"
#include "core/constraints/randomize_constraint.h"
#include "core/domain/mora_grid.h"

namespace {

bf::GraphEdge MakeEdge(int base_fl, int top_fl, bool is_high) {
  bf::GraphEdge e;
  e.to = 1;
  e.distance_nm = 100.0f;
  e.airway_id = 1;
  e.base_fl = static_cast<int16_t>(base_fl);
  e.top_fl = static_cast<int16_t>(top_fl);
  e.flags = is_high ? bf::kEdgeHigh : 0;
  return e;
}

// A request pinned to a single flight level (range collapsed to a point).
bf::RouteRequest WithAltitude(int fl) {
  bf::RouteRequest r;
  r.altitude = bf::FlRange{fl, fl};
  return r;
}

// A request with an inclusive cruise range [min_fl, max_fl].
bf::RouteRequest WithAltitudeRange(int min_fl, int max_fl) {
  bf::RouteRequest r;
  r.altitude = bf::FlRange{min_fl, max_fl};
  return r;
}

TEST_CASE("altitude band: no cruise altitude allows everything", "[constraint]") {
  bf::AltitudeBandConstraint c;
  bf::RouteRequest r;  // no altitude
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

TEST_CASE("altitude band: range overlapping the band is usable", "[constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitudeRange(300, 400)).allowed);  // fully inside
  CHECK(c.Evaluate(ctx, WithAltitudeRange(100, 200)).allowed);  // overlaps at base
  CHECK(c.Evaluate(ctx, WithAltitudeRange(400, 600)).allowed);  // overlaps at top
  CHECK(c.Evaluate(ctx, WithAltitudeRange(180, 180)).allowed);  // touches base only
  CHECK(c.Evaluate(ctx, WithAltitudeRange(450, 450)).allowed);  // touches top only
}

TEST_CASE("altitude band: range entirely outside the band is blocked", "[constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}};
  CHECK_FALSE(c.Evaluate(ctx, WithAltitudeRange(50, 170)).allowed);    // wholly below
  CHECK_FALSE(c.Evaluate(ctx, WithAltitudeRange(460, 600)).allowed);  // wholly above
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

TEST_CASE("MORA: range cleared when its top reaches the floor", "[constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(40, -74, 100);  // MORA FL100
  bf::MoraConstraint c(grid);
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{40.5, -73.5}};
  // Range top at/above MORA is allowed even if the bottom is below it: the
  // aircraft can hold a level within the range that clears terrain.
  CHECK(c.Evaluate(ctx, WithAltitudeRange(80, 120)).allowed);   // top clears
  CHECK(c.Evaluate(ctx, WithAltitudeRange(90, 100)).allowed);   // top at MORA
  CHECK_FALSE(c.Evaluate(ctx, WithAltitudeRange(60, 90)).allowed);  // whole range below
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

TEST_CASE("avoid: blocks edges entering an avoided vertex", "[constraint]") {
  bf::AvoidConstraint c({7}, {});  // avoid vertex 7
  bf::RouteRequest r;

  bf::GraphEdge into_7 = MakeEdge(0, 0, false);
  into_7.to = 7;
  CHECK_FALSE(c.Evaluate(bf::EdgeContext{into_7, bf::Coordinate{}}, r).allowed);

  bf::GraphEdge into_8 = MakeEdge(0, 0, false);
  into_8.to = 8;
  CHECK(c.Evaluate(bf::EdgeContext{into_8, bf::Coordinate{}}, r).allowed);
}

TEST_CASE("avoid: blocks edges on an avoided airway id", "[constraint]") {
  bf::AvoidConstraint c({}, {3});  // avoid airway_id 3
  bf::RouteRequest r;

  bf::GraphEdge on_3 = MakeEdge(0, 0, false);
  on_3.to = 1;
  on_3.airway_id = 3;
  CHECK_FALSE(c.Evaluate(bf::EdgeContext{on_3, bf::Coordinate{}}, r).allowed);

  bf::GraphEdge on_4 = MakeEdge(0, 0, false);
  on_4.to = 1;
  on_4.airway_id = 4;
  CHECK(c.Evaluate(bf::EdgeContext{on_4, bf::Coordinate{}}, r).allowed);
}

TEST_CASE("avoid: empty sets allow everything", "[constraint]") {
  bf::AvoidConstraint c({}, {});
  bf::RouteRequest r;
  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.to = 42;
  e.airway_id = 9;
  CHECK(c.Evaluate(bf::EdgeContext{e, bf::Coordinate{}}, r).allowed);
}

TEST_CASE("randomize: penalty is deterministic, non-negative, and bounded", "[constraint]") {
  bf::RouteRequest r;
  bf::GraphEdge e = MakeEdge(0, 0, false);  // distance_nm = 100
  e.to = 5;
  e.airway_id = 2;
  bf::EdgeContext ctx{e, bf::Coordinate{}};

  bf::RandomizeConstraint c(1234, 0.05);
  const bf::EdgeVerdict v1 = c.Evaluate(ctx, r);
  const bf::EdgeVerdict v2 = c.Evaluate(ctx, r);

  CHECK(v1.allowed);                              // never a hard filter
  CHECK(v1.extra_cost == v2.extra_cost);          // deterministic
  CHECK(v1.extra_cost >= 0.0);                    // non-negative (admissible)
  CHECK(v1.extra_cost <= 100.0 * 0.05 + 1e-9);    // bounded by eps * distance
}

TEST_CASE("randomize: different seeds generally differ; same seed matches", "[constraint]") {
  bf::RouteRequest r;
  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.to = 5;
  e.airway_id = 2;
  bf::EdgeContext ctx{e, bf::Coordinate{}};

  const double a = bf::RandomizeConstraint(1).Evaluate(ctx, r).extra_cost;
  const double b = bf::RandomizeConstraint(2).Evaluate(ctx, r).extra_cost;
  const double a_again = bf::RandomizeConstraint(1).Evaluate(ctx, r).extra_cost;
  CHECK(a == a_again);  // same seed, same edge => identical
  CHECK(a != b);        // different seeds => different perturbation (for this edge)
}

}  // namespace
