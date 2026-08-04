// SPDX-License-Identifier: MIT
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

#include "core/constraints/airway_rule_constraint.h"
#include "core/constraints/altitude_constraints.h"
#include "core/constraints/avoid_waypoint_constraint.h"
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
  e.level = is_high ? bf::AirwayLevel::kHigh : bf::AirwayLevel::kLow;
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

TEST_CASE("altitude band: no cruise altitude allows everything", "[unit][constraint]") {
  bf::AltitudeBandConstraint c;
  bf::RouteRequest r;  // no altitude
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, r).allowed);
}

TEST_CASE("altitude band: blocks outside the band", "[unit][constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitude(350)).allowed);        // inside
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(100)).allowed);  // below base
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(500)).allowed);  // above top
}

TEST_CASE("altitude band: range overlapping the band is usable", "[unit][constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitudeRange(300, 400)).allowed);  // fully inside
  CHECK(c.Evaluate(ctx, WithAltitudeRange(100, 200)).allowed);  // overlaps at base
  CHECK(c.Evaluate(ctx, WithAltitudeRange(400, 600)).allowed);  // overlaps at top
  CHECK(c.Evaluate(ctx, WithAltitudeRange(180, 180)).allowed);  // touches base only
  CHECK(c.Evaluate(ctx, WithAltitudeRange(450, 450)).allowed);  // touches top only
}

TEST_CASE("altitude band: range entirely outside the band is blocked", "[unit][constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(180, 450, true), bf::Coordinate{}, bf::Coordinate{}};
  CHECK_FALSE(c.Evaluate(ctx, WithAltitudeRange(50, 170)).allowed);   // wholly below
  CHECK_FALSE(c.Evaluate(ctx, WithAltitudeRange(460, 600)).allowed);  // wholly above
}

TEST_CASE("altitude band: zero band (DCT) is exempt", "[unit][constraint]") {
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitude(350)).allowed);
}

TEST_CASE("altitude band: an open (0) ceiling imposes no upper bound", "[unit][constraint]") {
  // A high-altitude segment with a floor but no published ceiling (top_fl == 0
  // means "open"). A cruise level above the floor must be allowed, not blocked as
  // if the range lay above a real ceiling of 0.
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(200, 0, true), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitude(400)).allowed);        // well above the floor
  CHECK(c.Evaluate(ctx, WithAltitude(200)).allowed);        // at the floor
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(100)).allowed);  // below the floor
}

TEST_CASE("altitude band: an open (0) floor imposes no lower bound", "[unit][constraint]") {
  // Symmetric: base_fl == 0 is an open floor, only the ceiling constrains.
  bf::AltitudeBandConstraint c;
  bf::EdgeContext ctx{MakeEdge(0, 180, false), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(ctx, WithAltitude(50)).allowed);         // low, still under the ceiling
  CHECK(c.Evaluate(ctx, WithAltitude(180)).allowed);        // at the ceiling
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(250)).allowed);  // above the ceiling
}

TEST_CASE("level preference: penalizes non-preferred level", "[unit][constraint]") {
  bf::LevelPreferenceConstraint c(0.5);
  bf::RouteRequest r;
  r.level = bf::LevelPreference::kHigh;

  bf::EdgeContext high{MakeEdge(180, 450, true), bf::Coordinate{}, bf::Coordinate{}};
  bf::EdgeContext low{MakeEdge(0, 180, false), bf::Coordinate{}, bf::Coordinate{}};
  CHECK(c.Evaluate(high, r).extra_cost == 0.0);  // preferred: no penalty
  bf::EdgeVerdict v = c.Evaluate(low, r);
  CHECK(v.allowed);             // not forbidden
  CHECK(v.extra_cost == 50.0);  // 100 NM * 0.5
}

TEST_CASE("level preference: both-level edge never penalized", "[unit][constraint]") {
  bf::LevelPreferenceConstraint c(0.5);
  bf::GraphEdge e = MakeEdge(180, 450, true);
  e.level = bf::AirwayLevel::kBoth;  // DFD flightlevel 'B': usable at either level
  bf::EdgeContext ctx{e, bf::Coordinate{}, bf::Coordinate{}};

  bf::RouteRequest high;
  high.level = bf::LevelPreference::kHigh;
  CHECK(c.Evaluate(ctx, high).extra_cost == 0.0);

  bf::RouteRequest low;
  low.level = bf::LevelPreference::kLow;
  CHECK(c.Evaluate(ctx, low).extra_cost == 0.0);
}

TEST_CASE("MORA: blocks below grid minimum, allows at or above", "[unit][constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(40, -74, 100);  // cell covering ~JFK area: MORA FL100
  bf::MoraConstraint c(grid);

  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{40.5, -73.5},
                      bf::Coordinate{40.5, -73.5}};
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(80)).allowed);  // below MORA
  CHECK(c.Evaluate(ctx, WithAltitude(100)).allowed);       // at MORA
  CHECK(c.Evaluate(ctx, WithAltitude(150)).allowed);       // above MORA
}

TEST_CASE("MORA: range cleared when its top reaches the floor", "[unit][constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(40, -74, 100);  // MORA FL100
  bf::MoraConstraint c(grid);
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{40.5, -73.5},
                      bf::Coordinate{40.5, -73.5}};
  // Range top at/above MORA is allowed even if the bottom is below it: the
  // aircraft can hold a level within the range that clears terrain.
  CHECK(c.Evaluate(ctx, WithAltitudeRange(80, 120)).allowed);       // top clears
  CHECK(c.Evaluate(ctx, WithAltitudeRange(90, 100)).allowed);       // top at MORA
  CHECK_FALSE(c.Evaluate(ctx, WithAltitudeRange(60, 90)).allowed);  // whole range below
}

TEST_CASE("MORA: unknown cell imposes no limit", "[unit][constraint]") {
  bf::MoraGrid grid;  // empty
  bf::MoraConstraint c(grid);
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{10.0, 20.0},
                      bf::Coordinate{10.0, 20.0}};
  CHECK(c.Evaluate(ctx, WithAltitude(50)).allowed);
}

TEST_CASE("MORA: samples the short way across the antimeridian", "[unit][constraint]") {
  bf::MoraGrid grid;
  // A high floor on the far side of the globe (lon 0). A correct short-path
  // sampler for a +179 -> -179 leg must NOT walk through it; the old long-way
  // interpolation swept the whole globe and would wrongly pick it up, blocking a
  // legitimate trans-Pacific leg.
  grid.SetCell(0, 0, 300);
  bf::MoraConstraint c(grid);
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{0.5, 179.2},
                      bf::Coordinate{0.5, -179.4}};
  CHECK(c.Evaluate(ctx, WithAltitude(200)).allowed);  // no floor on the true track
}

TEST_CASE("MORA: a high cell on the antimeridian crossing still blocks", "[unit][constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(0, 179, 300);  // a cell the true short track actually passes through
  bf::MoraConstraint c(grid);
  bf::EdgeContext ctx{MakeEdge(0, 0, false), bf::Coordinate{0.5, 179.2},
                      bf::Coordinate{0.5, -179.4}};
  CHECK_FALSE(c.Evaluate(ctx, WithAltitude(200)).allowed);  // below the FL300 floor
}

TEST_CASE("MORA grid floors negative coordinates correctly", "[unit][constraint]") {
  bf::MoraGrid grid;
  grid.SetCell(-1, -1, 50);
  // A point at (-0.5, -0.5) floors to cell (-1, -1).
  CHECK(grid.MoraAt(bf::Coordinate{-0.5, -0.5}) == 50);
  // A point at (0.5, 0.5) floors to cell (0, 0): unset.
  CHECK(grid.MoraAt(bf::Coordinate{0.5, 0.5}) == 0);
}

TEST_CASE("avoid: blocks edges entering an avoided vertex", "[unit][constraint]") {
  bf::AvoidWaypointConstraint c({7});  // avoid vertex 7
  bf::RouteRequest r;

  bf::GraphEdge into_7 = MakeEdge(0, 0, false);
  into_7.to = 7;
  CHECK_FALSE(c.Evaluate(bf::EdgeContext{into_7, bf::Coordinate{}, bf::Coordinate{}}, r).allowed);

  bf::GraphEdge into_8 = MakeEdge(0, 0, false);
  into_8.to = 8;
  CHECK(c.Evaluate(bf::EdgeContext{into_8, bf::Coordinate{}, bf::Coordinate{}}, r).allowed);
}

TEST_CASE("avoid: empty set allows everything", "[unit][constraint]") {
  bf::AvoidWaypointConstraint c({});
  bf::RouteRequest r;
  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.to = 42;
  e.airway_id = 9;
  CHECK(c.Evaluate(bf::EdgeContext{e, bf::Coordinate{}, bf::Coordinate{}}, r).allowed);
}

TEST_CASE("randomize: penalty is deterministic, non-negative, and bounded", "[unit][constraint]") {
  bf::RouteRequest r;
  bf::GraphEdge e = MakeEdge(0, 0, false);  // distance_nm = 100
  e.to = 5;
  e.airway_id = 2;
  bf::EdgeContext ctx{e, bf::Coordinate{}, bf::Coordinate{}};

  bf::RandomizeConstraint c(1234, 0.05);
  const bf::EdgeVerdict v1 = c.Evaluate(ctx, r);
  const bf::EdgeVerdict v2 = c.Evaluate(ctx, r);

  CHECK(v1.allowed);                            // never a hard filter
  CHECK(v1.extra_cost == v2.extra_cost);        // deterministic
  CHECK(v1.extra_cost >= 0.0);                  // non-negative (admissible)
  CHECK(v1.extra_cost <= 100.0 * 0.05 + 1e-9);  // bounded by eps * distance
}

TEST_CASE("randomize: different seeds generally differ; same seed matches", "[unit][constraint]") {
  bf::RouteRequest r;
  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.to = 5;
  e.airway_id = 2;
  bf::EdgeContext ctx{e, bf::Coordinate{}, bf::Coordinate{}};

  const double a = bf::RandomizeConstraint(1).Evaluate(ctx, r).extra_cost;
  const double b = bf::RandomizeConstraint(2).Evaluate(ctx, r).extra_cost;
  const double a_again = bf::RandomizeConstraint(1).Evaluate(ctx, r).extra_cost;
  CHECK(a == a_again);  // same seed, same edge => identical
  CHECK(a != b);        // different seeds => different perturbation (for this edge)
}

// --- airway rules ----------------------------------------------------------
// The constraint itself consumes resolved bitmask tables, so these tests build a
// tiny two-vertex / three-airway "graph" by hand: vertex 0 matches rule 0, vertex 1
// matches rule 1, and so on. The prefix-matching helpers are exercised directly.

TEST_CASE("matches any prefix: empty list and empty entry match everything", "[unit][constraint]") {
  CHECK(bf::MatchesAnyPrefix("ZB", {}));         // no prefixes => any region
  CHECK(bf::MatchesAnyPrefix("ZB", {""}));       // an empty entry => any region
  CHECK(bf::MatchesAnyPrefix("", {}));           // an empty region is fine too
  CHECK_FALSE(bf::MatchesAnyPrefix("", {"Z"}));  // ...but matches no real prefix
}

TEST_CASE("matches any prefix: prefix semantics across regions", "[unit][constraint]") {
  // "Z" is the whole Z-block, including ZM (Mongolia) and ZK (North Korea) -- the
  // reason a rule can enumerate regions instead.
  CHECK(bf::MatchesAnyPrefix("ZB", {"Z"}));
  CHECK(bf::MatchesAnyPrefix("ZM", {"Z"}));
  // A two-char entry is effectively exact: nothing longer extends it.
  CHECK(bf::MatchesAnyPrefix("ZB", {"ZB"}));
  CHECK_FALSE(bf::MatchesAnyPrefix("ZM", {"ZB"}));
  // An enumerated set matches each member and nothing else.
  const std::vector<std::string> china = {"ZB", "ZG", "ZS"};
  CHECK(bf::MatchesAnyPrefix("ZB", china));
  CHECK(bf::MatchesAnyPrefix("ZS", china));
  CHECK_FALSE(bf::MatchesAnyPrefix("ZM", china));
}

TEST_CASE("matches any designator: exact mode does not catch extensions", "[unit][constraint]") {
  using Match = bf::AirwayRule::Match;
  // The whole reason exact mode exists: 1371 designators are a strict prefix of
  // another one, so exact "J60" must not match J603.
  CHECK(bf::MatchesAnyDesignator("J60", {"J60"}, Match::kExact));
  CHECK_FALSE(bf::MatchesAnyDesignator("J603", {"J60"}, Match::kExact));
  // Prefix mode is the category rule and deliberately does catch them.
  CHECK(bf::MatchesAnyDesignator("J60", {"J"}, Match::kPrefix));
  CHECK(bf::MatchesAnyDesignator("J603", {"J"}, Match::kPrefix));
  CHECK_FALSE(bf::MatchesAnyDesignator("V16", {"J"}, Match::kPrefix));
  // An empty list matches everything in either mode.
  CHECK(bf::MatchesAnyDesignator("J60", {}, Match::kExact));
  CHECK(bf::MatchesAnyDesignator("J60", {}, Match::kPrefix));
  // A list shares one rule, so any member hitting is a hit.
  CHECK(bf::MatchesAnyDesignator("A3", {"J60", "A3", "W19"}, Match::kExact));
  CHECK_FALSE(bf::MatchesAnyDesignator("A30", {"J60", "A3", "W19"}, Match::kExact));
}

TEST_CASE("airway rules: a leg matches when either endpoint's region matches",
          "[unit][constraint]") {
  bf::RouteRequest r;
  // One rule (bit 0). Vertex 1 is in a matching region, vertices 0 and 2 are not;
  // airway 1 matches the designator side.
  bf::AirwayRuleConstraint c(/*vertex_mask=*/{0b0, 0b1, 0b0},
                             /*airway_mask=*/{0b0, 0b1}, /*block_bits=*/0b1,
                             /*fractions=*/{0.0});

  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.airway_id = 1;
  // 0 -> 1: the destination is in the region.
  e.to = 1;
  CHECK_FALSE(c.Evaluate(bf::EdgeContext{e, bf::Coordinate{}, bf::Coordinate{}, 0}, r).allowed);
  // 1 -> 2: the source is in the region. This is the case that needs
  // EdgeContext::from -- GraphEdge alone only knows `to`.
  e.to = 2;
  CHECK_FALSE(c.Evaluate(bf::EdgeContext{e, bf::Coordinate{}, bf::Coordinate{}, 1}, r).allowed);
  // 0 -> 2: neither endpoint is in the region, so the rule does not apply.
  e.to = 2;
  CHECK(c.Evaluate(bf::EdgeContext{e, bf::Coordinate{}, bf::Coordinate{}, 0}, r).allowed);
}

TEST_CASE("airway rules: DCT edges are never ruled on", "[unit][constraint]") {
  bf::RouteRequest r;
  // airway_mask[0] is always 0: airway id 0 is the reserved synthetic "DCT" name.
  bf::AirwayRuleConstraint c({0b1, 0b1}, {0b0, 0b1}, 0b1, {0.0});
  bf::GraphEdge dct = MakeEdge(0, 0, false);
  dct.airway_id = 0;
  dct.to = 1;
  CHECK(c.Evaluate(bf::EdgeContext{dct, bf::Coordinate{}, bf::Coordinate{}, 0}, r).allowed);
}

TEST_CASE("airway rules: block wins over a matching penalize rule", "[unit][constraint]") {
  bf::RouteRequest r;
  // Two rules both matching this leg: bit 0 penalizes, bit 1 blocks.
  bf::AirwayRuleConstraint c({0b11, 0b11}, {0b0, 0b11}, /*block_bits=*/0b10, {0.5, 0.0});
  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.airway_id = 1;
  e.to = 1;
  const bf::EdgeVerdict v = c.Evaluate(bf::EdgeContext{e, {}, {}, 0}, r);
  CHECK_FALSE(v.allowed);
  CHECK(v.extra_cost == 0.0);  // a blocked edge carries no penalty
}

TEST_CASE("airway rules: penalties of several matching rules sum", "[unit][constraint]") {
  bf::RouteRequest r;
  // Both rules penalize: 0.8 ("all V airways") + 0.3 ("V airways in this region").
  bf::AirwayRuleConstraint c({0b11, 0b11}, {0b0, 0b11}, /*block_bits=*/0, {0.8, 0.3});
  bf::GraphEdge e = MakeEdge(0, 0, false);  // distance_nm = 100
  e.airway_id = 1;
  e.to = 1;
  const bf::EdgeVerdict v = c.Evaluate(bf::EdgeContext{e, {}, {}, 0}, r);
  CHECK(v.allowed);  // penalize keeps the leg usable
  // Sums past 1.0 on purpose: stacking rules toward block-like strength is allowed.
  CHECK(v.extra_cost == Catch::Approx(100.0 * 1.1));
}

TEST_CASE("airway rules: a rule matching only the designator side does not apply",
          "[unit][constraint]") {
  bf::RouteRequest r;
  // The designator matches (airway 1) but no vertex is in the region.
  bf::AirwayRuleConstraint c({0b0, 0b0}, {0b0, 0b1}, 0b1, {0.0});
  bf::GraphEdge e = MakeEdge(0, 0, false);
  e.airway_id = 1;
  e.to = 1;
  CHECK(c.Evaluate(bf::EdgeContext{e, {}, {}, 0}, r).allowed);
}

}  // namespace
