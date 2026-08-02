// SPDX-License-Identifier: MIT
// Integration tests for the Fenix A320 SQLite loader. Real Jeppesen data is
// never committed; each case SKIPs when the data is absent. To avoid loading
// the full 329k-waypoint dataset per-check, validations are grouped into
// SECTIONS under a shared LoadNavData call.

#include "io/loaders/fenix/fenix_loader.h"

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include "core/base/env.h"
#include "io/loaders/dfd1/dfd1_loader.h"
#include "io/loaders/loader_registry.h"
#include "test_db.h"

namespace {

std::string EnsureFenix() {
  // Pin to the per-loader subdirectory, mirroring EnsureDfd1 / EnsureXPlane12.
  bf::test::SetNavDataDir("navdata/fenix");
  const std::string dir = bf::test::NavDataDir();
  return bf::test::HasDb3(dir) ? dir : std::string{};
}

std::string EnsureDfd1ForFenix() {
  // Pin to the per-loader subdirectory; the cross-loader passes this dir
  // straight to Dfd1Loader, so it must resolve the real DFD v1 data (not the
  // top-level 0-byte placeholders) regardless of BRAVOFINDER_NAVDATA.
  bf::test::SetNavDataDir("navdata/dfd1");
  const std::string dir = bf::test::NavDataDir();
  return bf::test::HasS3db(dir) ? dir : std::string{};
}

// Load enroute data once, validate in SECTIONS.
struct FenixData {
  bf::NavData data;
  explicit FenixData(const std::string& dir) {
    bf::FenixLoader l;
    auto r = l.LoadNavData(dir);
    if (r) data = std::move(r.value());
  }
  explicit operator bool() const { return !data.waypoints.empty(); }
};

}  // namespace

TEST_CASE("fenix: registry resolves", "[integration][fenix]") {
  auto r = bf::MakeLoader("fenix");
  REQUIRE(r);
  CHECK(r.value()->name() == "fenix");
}

TEST_CASE("fenix: LoadNavData enroute dataset", "[integration][fenix]") {
  const std::string dir = EnsureFenix();
  if (dir.empty()) SKIP("Fenix navdata not found");
  FenixData fd(dir);
  REQUIRE(fd);

  SECTION("counts") {
    CHECK(fd.data.waypoints.size() > 200000);
    CHECK(fd.data.airways.size() > 80000);
    CHECK(fd.data.airports.size() > 10000);
    CHECK(fd.data.hold_fixes.size() > 20000);
    CHECK(fd.data.navaid_details.size() > 5000);
    CHECK(!fd.data.mora.Empty());
    CHECK(fd.data.msa.empty());
  }
  SECTION("waypoint coords") {
    int bad = 0, reg_long = 0;
    for (const auto& w : fd.data.waypoints) {
      if (w.coord.latitude < -90 || w.coord.latitude > 90) ++bad;
      if (w.coord.longitude < -180 || w.coord.longitude > 180) ++bad;
      if (w.ident.arinc424_icao_code.size() > 2) ++reg_long;
    }
    CHECK(bad == 0);
    CHECK(reg_long == 0);
  }
  SECTION("navaid kinds") {
    int v = 0, n = 0, d = 0;
    for (const auto& w : fd.data.waypoints) {
      if (w.kind == bf::WaypointKind::kVor)
        ++v;
      else if (w.kind == bf::WaypointKind::kNdb)
        ++n;
      else if (w.kind == bf::WaypointKind::kDme)
        ++d;
      if (v > 3000 && n > 3000 && d > 1000) break;  // early exit
    }
    // kVor = VOR(120)+VORTAC(474)+VOR-DME(3112) ≈ 3706
    // kNdb = NDB(3083)+NDB-DME(20) ≈ 3103
    // kDme = TACAN(433)+DME excl ILS(820) ≈ 1253
    CHECK(v > 3000);
    CHECK(n > 3000);
    CHECK(d > 1000);
  }
  SECTION("navaid freq BCD decode") {
    // Fenix stores freq as 8-digit packed BCD (freq*10000); the loader must
    // decode it to the freq_raw contract (kHz for NDB, MHz*100 otherwise).
    // Before the fix ColumnInt was stored verbatim, so every freq was ~1000x
    // off and landed far outside the aviation bands.
    const auto find = [](const std::vector<bf::NavaidDetail>& v, const std::string& id,
                         bf::WaypointKind k) -> const bf::NavaidDetail* {
      for (const auto& d : v)
        if (d.ident.ident == id && d.kind == k) return &d;
      return nullptr;
    };
    // LAX VORTAC 113.6 MHz -> 11360; ATL NDB 422.0 kHz -> 422.
    const auto* lax = find(fd.data.navaid_details, "LAX", bf::WaypointKind::kVor);
    REQUIRE(lax != nullptr);
    CHECK(lax->freq_raw == 11360);
    const auto* atl = find(fd.data.navaid_details, "ATL", bf::WaypointKind::kNdb);
    REQUIRE(atl != nullptr);
    CHECK(atl->freq_raw == 422);
    // Decoded VOR/DME/ILS freqs must land in the VHF band; NDBs in the MF band.
    int bad = 0;
    for (const auto& d : fd.data.navaid_details) {
      if (d.kind == bf::WaypointKind::kNdb) {
        if (d.freq_raw < 100 || d.freq_raw > 2000) ++bad;
      } else if (d.kind == bf::WaypointKind::kVor || d.kind == bf::WaypointKind::kDme ||
                 d.kind == bf::WaypointKind::kOther) {
        const double mhz = d.freq_raw / 100.0;
        if (mhz < 108.0 || mhz > 136.0) ++bad;
      }
    }
    CHECK(bad == 0);
  }
  SECTION("AROKE") {
    auto it = std::find_if(fd.data.waypoints.begin(), fd.data.waypoints.end(),
                           [](auto& w) { return w.ident.ident == "AROKE"; });
    REQUIRE(it != fd.data.waypoints.end());
    CHECK(it->coord.latitude == Catch::Approx(40.4724).margin(0.01));
    CHECK(it->coord.longitude == Catch::Approx(-73.9021).margin(0.01));
  }
  SECTION("airways") {
    int lo = 0, hi = 0, bo = 0, notboth = 0;
    for (const auto& c : fd.data.airways) {
      if (c.segment.level == bf::AirwayLevel::kLow)
        ++lo;
      else if (c.segment.level == bf::AirwayLevel::kHigh)
        ++hi;
      else if (c.segment.level == bf::AirwayLevel::kBoth)
        ++bo;
      if (c.segment.direction != bf::AirwayDirection::kBoth) ++notboth;
    }
    CHECK(lo > 50000);
    CHECK(hi > 40000);
    CHECK(bo > 10000);
    CHECK(notboth == 0);
  }
  SECTION("airports") {
    auto it = std::find_if(fd.data.airports.begin(), fd.data.airports.end(),
                           [](auto& a) { return a.icao == "KJFK"; });
    REQUIRE(it != fd.data.airports.end());
    CHECK(it->coord.latitude == Catch::Approx(40.64).margin(0.1));
    CHECK(it->coord.longitude == Catch::Approx(-73.78).margin(0.1));
    CHECK(it->elevation_ft == Catch::Approx(13).margin(5));
  }
  SECTION("holds") {
    int enrt = 0, term = 0;
    for (const auto& h : fd.data.hold_fixes) {
      if (h.airport_icao == "ENRT")
        ++enrt;
      else
        ++term;
      if (enrt > 1000 && term > 1000) break;
    }
    CHECK(enrt > 1000);
    CHECK(term > 1000);
  }
  SECTION("MORA") {
    // MORA cells are flight levels (hundreds of feet) keyed by signed geographic
    // degrees. Two regressions to guard against:
    //   * H1: a +180/+360 remap once pushed the whole south/west hemisphere out
    //     of range, silently dropping it -- so cells below the equator / west of
    //     the prime meridian must be populated.
    //   * H2: a *100 amplification once made every value 100x too large (and
    //     overflowed int16_t on high-terrain cells). A real MORA never reaches
    //     600 (60000 ft -- far above any terrain), so the grid max staying below
    //     that bound catches the amplification.
    int16_t max_cell = 0;
    int south_pop = 0, west_pop = 0;
    for (int lat = -90; lat <= 89; ++lat) {
      for (int lon = -180; lon <= 179; ++lon) {
        const int16_t v = fd.data.mora.MoraAt(
            bf::Coordinate{static_cast<double>(lat) + 0.5, static_cast<double>(lon) + 0.5});
        if (v <= 0) continue;
        max_cell = std::max(max_cell, v);
        if (lat < 0) ++south_pop;
        if (lon < 0) ++west_pop;
      }
    }
    CHECK(south_pop > 0);   // H1: southern hemisphere populated
    CHECK(west_pop > 0);    // H1: western hemisphere populated
    CHECK(max_cell < 600);  // H2: no 100x amplification
    // NE quadrant (the only quadrant the old bug left intact) is still populated.
    CHECK(fd.data.mora.MoraAt(bf::Coordinate{28.0, 87.0}) > 0);
  }
}

TEST_CASE("fenix: LoadProcedures", "[integration][fenix]") {
  const std::string dir = EnsureFenix();
  if (dir.empty()) SKIP("Fenix navdata not found");
  bf::FenixLoader l;
  auto r = l.LoadProcedures(dir);
  REQUIRE(r);
  CHECK(r.value().size() > 10000);
  auto it =
      std::find_if(r.value().begin(), r.value().end(), [](auto& a) { return a.first == "KJFK"; });
  REQUIRE(it != r.value().end());
  REQUIRE(!it->second.procedures.empty());
}

TEST_CASE("fenix: LoadProcedure KJFK", "[integration][fenix]") {
  const std::string dir = EnsureFenix();
  if (dir.empty()) SKIP("Fenix navdata not found");
  bf::FenixLoader l;
  auto r = l.LoadProcedure(dir, "KJFK");
  REQUIRE(r.has_value());
  CHECK(!r->procedures.empty());
  CHECK(!r->runways.empty());
  CHECK_FALSE(l.LoadProcedure(dir, "ZZZZ").has_value());
  int tf = 0, unk = 0, alt = 0;
  for (const auto& p : r->procedures)
    for (const auto& leg : p.legs) {
      if (leg.path_term == bf::PathTerminator::kTF)
        ++tf;
      else if (leg.path_term == bf::PathTerminator::kUnknown)
        ++unk;
      if (leg.alt.kind != bf::AltConstraintKind::kNone) ++alt;
    }
  CHECK(tf > 50);
  CHECK(unk == 0);
  CHECK(alt > 10);
}

TEST_CASE("fenix: Procedure::runway is RW-prefixed only", "[integration][fenix]") {
  const std::string dir = EnsureFenix();
  if (dir.empty()) SKIP("Fenix navdata not found");
  bf::FenixLoader l;
  auto r = l.LoadProcedure(dir, "KJFK");
  REQUIRE(r.has_value());

  // Find a procedure record by (name, transition_ident). KJFK anchors verified
  // against navdata/fenix/nd.db3 (cycle 2601).
  auto find = [&](const std::string& name, const std::string& trans) -> const bf::Procedure* {
    for (const auto& p : r->procedures) {
      if (p.name == name && p.transition_ident == trans) return &p;
    }
    return nullptr;
  };

  // (a) A runway transition (RW-prefixed ident) carries the runway.
  const bf::Procedure* sid_rwy = find("DEEZZ5", "RW31L");
  REQUIRE(sid_rwy != nullptr);
  CHECK(sid_rwy->type == bf::ProcedureType::kSid);
  CHECK(sid_rwy->runway == "RW31L");

  // (b) An enroute transition (fix name) carries no runway: under --rwy-dep it
  // must stay selectable, not be wrongly excluded by RunwayMatches.
  const bf::Procedure* sid_enr = find("DEEZZ5", "CANDR");
  REQUIRE(sid_enr != nullptr);
  CHECK(sid_enr->runway.empty());

  // (c) An approach IAF transition (fix name) carries no runway -- approaches
  // never have an RW-prefixed transition in Fenix, so their runway is always
  // empty, matching DFD1/DFD2/X-Plane.
  const bf::Procedure* apch_iaf = find("I13L", "COVIR");
  REQUIRE(apch_iaf != nullptr);
  CHECK(apch_iaf->type == bf::ProcedureType::kApproach);
  CHECK(apch_iaf->runway.empty());

  // Global invariant: runway is either empty or an RW-prefixed ident. The
  // pre-fix loader populated ~120k records with a bogus fix-name runway.
  for (const auto& p : r->procedures) {
    CHECK((p.runway.empty() || p.runway.rfind("RW", 0) == 0));
  }
}

TEST_CASE("fenix/dfd1: cross-loader AROKE", "[integration][fenix]") {
  auto fd = EnsureFenix(), dd = EnsureDfd1ForFenix();
  if (fd.empty() || dd.empty()) SKIP("data missing");
  bf::FenixLoader fl;
  bf::Dfd1Loader dl;
  auto fn = fl.LoadNavData(fd), dn = dl.LoadNavData(dd);
  REQUIRE(fn);
  REQUIRE(dn);
  auto f = [](auto& wps, auto& id) {
    for (auto& w : wps)
      if (w.ident.ident == id) return &w;
    return (bf::Waypoint*)nullptr;
  };
  auto fa = f(fn.value().waypoints, "AROKE"), da = f(dn.value().waypoints, "AROKE");
  REQUIRE(fa);
  REQUIRE(da);
  CHECK(std::fabs(fa->coord.latitude - da->coord.latitude) < 1e-4);
  CHECK(std::fabs(fa->coord.longitude - da->coord.longitude) < 1e-4);
}
