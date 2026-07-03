#include "io/cache/nav_detail_cache.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <string>
#include <vector>

#include "core/domain/hold_fix.h"
#include "core/domain/navaid_detail.h"
#include "io/nav_data.h"

namespace {

// A temp path unique to this test file; removed at the end of each case.
std::string TempPath(const std::string& tag) {
  return std::string("/tmp/bravofinder_nav_detail_test_") + tag + ".bfdb";
}

// Build a small NavData with a couple of navaids (one sharing an ident across
// regions) and a couple of holds (one fix carrying two holds), enough to
// exercise the sorted-array multi-value lookups.
bf::NavData MakeSampleData() {
  bf::NavData d;
  d.cycle = 2601;
  d.build = 20260112;

  d.navaid_details.push_back(
      bf::NavaidDetail{bf::Ident("SEA", "K1"), bf::WaypointKind::kVor, 354, 11680, 150.0, 19.0});
  d.navaid_details.push_back(
      bf::NavaidDetail{bf::Ident("DGC", "K2"), bf::WaypointKind::kVor, 500, 11500, 130.0, -3.0});
  // Same ident, different region -> both must come back from a bare-ident query.
  d.navaid_details.push_back(
      bf::NavaidDetail{bf::Ident("DGC", "LF"), bf::WaypointKind::kNdb, 0, 350, 50.0, 0.0});

  bf::HoldFix h1;
  h1.fix = bf::Ident("AE701", "DA");
  h1.airport_icao = "DAAE";
  h1.inbound_course = 171.0;
  h1.leg_time_min = 1.0;
  h1.leg_dist_nm = 0.0;
  h1.turn_dir = 'R';
  h1.min_alt_ft = 5580;
  h1.max_alt_ft = 14000;
  h1.speed_limit_kt = 230;
  d.hold_fixes.push_back(h1);

  // A second hold at the same fix ident (different parameters) to exercise the
  // multi-value equal-range lookup.
  bf::HoldFix h2 = h1;
  h2.inbound_course = 351.0;
  h2.turn_dir = 'L';
  d.hold_fixes.push_back(h2);

  // An enroute hold at a different fix.
  bf::HoldFix h3;
  h3.fix = bf::Ident("BOTON", "LF");
  h3.airport_icao = "ENRT";
  h3.inbound_course = 90.0;
  h3.leg_time_min = 0.0;
  h3.leg_dist_nm = 7.0;
  h3.turn_dir = 'R';
  h3.min_alt_ft = 3000;
  h3.max_alt_ft = 0;  // no upper limit
  h3.speed_limit_kt = 0;
  d.hold_fixes.push_back(h3);

  return d;
}

}  // namespace

TEST_CASE("nav detail cache: build -> open round-trips navaids", "[unit][detail]") {
  const std::string path = TempPath("navaids");
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(MakeSampleData());
  REQUIRE(bf::NavDetailCache::Build(path, src, "xplane", "3.2.0"));

  bf::Result<bf::NavDetailArchive> opened = bf::NavDetailCache::Open(path);
  REQUIRE(opened);
  const bf::NavDetailArchive& a = opened.value();
  CHECK(a.cycle() == 2601);
  CHECK(a.build() == 20260112);

  // Single-region navaid.
  auto sea = a.FindNavaids("SEA");
  REQUIRE(sea.size() == 1);
  CHECK(sea[0].ident == "SEA");
  CHECK(sea[0].region == "K1");
  CHECK(sea[0].kind == bf::WaypointKind::kVor);
  CHECK(sea[0].elev_ft == 354);
  CHECK(sea[0].freq_raw == 11680);
  CHECK(sea[0].range_nm == 150.0);
  CHECK(sea[0].heading == 19.0);

  // Ident reused across regions returns both matches.
  auto dgc = a.FindNavaids("DGC");
  REQUIRE(dgc.size() == 2);

  // Unknown ident -> empty.
  CHECK(a.FindNavaids("NOPE").empty());

  std::remove(path.c_str());
}

TEST_CASE("nav detail cache: build -> open round-trips holds", "[unit][detail]") {
  const std::string path = TempPath("holds");
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(MakeSampleData());
  REQUIRE(bf::NavDetailCache::Build(path, src, "xplane", "3.2.0"));

  bf::Result<bf::NavDetailArchive> opened = bf::NavDetailCache::Open(path);
  REQUIRE(opened);
  const bf::NavDetailArchive& a = opened.value();

  // A fix ident with two holds returns both (multi-value equal-range).
  auto ae = a.FindHolds("AE701");
  REQUIRE(ae.size() == 2);
  for (const bf::HoldInfo& h : ae) {
    CHECK(h.fix_ident == "AE701");
    CHECK(h.fix_region == "DA");
    CHECK(h.airport_icao == "DAAE");
    CHECK(h.min_alt_ft == 5580);
    CHECK(h.max_alt_ft == 14000);
    CHECK(h.speed_limit_kt == 230);
  }
  // The two differ in inbound course and turn direction.
  CHECK(ae[0].turn_dir != ae[1].turn_dir);

  // An enroute hold with no upper limit / no speed limit.
  auto boton = a.FindHolds("BOTON");
  REQUIRE(boton.size() == 1);
  CHECK(boton[0].airport_icao == "ENRT");
  CHECK(boton[0].leg_dist_nm == 7.0);
  CHECK(boton[0].max_alt_ft == 0);
  CHECK(boton[0].speed_limit_kt == 0);

  CHECK(a.FindHolds("NOPE").empty());

  std::remove(path.c_str());
}

TEST_CASE("nav detail cache: open rejects a bad magic", "[unit][detail]") {
  const std::string path = TempPath("badmagic");
  {
    FILE* f = std::fopen(path.c_str(), "wb");
    REQUIRE(f != nullptr);
    std::fputs("XXXX and some trailing bytes", f);
    std::fclose(f);
  }
  bf::Result<bf::NavDetailArchive> opened = bf::NavDetailCache::Open(path);
  CHECK_FALSE(opened);
  std::remove(path.c_str());
}

TEST_CASE("nav detail cache: empty data round-trips to an empty archive", "[unit][detail]") {
  const std::string path = TempPath("empty");
  bf::NavData empty;
  empty.cycle = 2601;
  empty.build = 20260112;
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(empty);
  REQUIRE(bf::NavDetailCache::Build(path, src, "xplane", "3.2.0"));

  bf::Result<bf::NavDetailArchive> opened = bf::NavDetailCache::Open(path);
  REQUIRE(opened);
  CHECK(opened.value().FindNavaids("SEA").empty());
  CHECK(opened.value().FindHolds("AE701").empty());
  std::remove(path.c_str());
}
