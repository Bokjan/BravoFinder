// Integration tests for the DFD SQLite loaders (DFD v1.0 and DFD v2). Uses the
// real cycle-2601 Navigraph SQLite databases (PMDG e_dfd_PMDG.s3db for v1,
// Inibuilds db.s3db for v2) located under navdata/dfd1/ and navdata/dfd2/. Real
// Jeppesen data is never committed, so each case SKIPs when the data is absent
// (CLAUDE.md: real data, no mocks).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

#include "io/loaders/dfd1/dfd1_loader.h"
#include "io/loaders/dfd2/dfd2_loader.h"
#include "io/loaders/loader_registry.h"
#include "io/loaders/xplane12/cifp/cifp_parser.h"
#include "test_dfd1.h"
#include "test_dfd2.h"
#include "test_xplane12.h"

namespace {

using bf::test::EnsureDfd1;
using bf::test::EnsureDfd2;
using bf::test::EnsureXPlane12;
using bf::test::HasCifp;

// The AIRAC cycle of the bundled 2601 data. Both DFD versions carry the same
// cycle; loaders extract it from their respective header tables.
constexpr uint32_t kExpectedCycle = 2601;

}  // namespace

// ---------------------------------------------------------------------------
// DFD v1.0 (Dfd1Loader)
// ---------------------------------------------------------------------------

TEST_CASE("dfd1: registry resolves and names itself", "[integration][dfd]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("dfd1");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "dfd1");
}

TEST_CASE("dfd1: LoadNavData parses the enroute dataset", "[integration][dfd]") {
  const std::string dir = EnsureDfd1();
  if (dir.empty()) {
    SKIP("DFD v1 data not found (set up navdata_dfd1/ with a .s3db)");
  }
  bf::Dfd1Loader loader;
  bf::Result<bf::NavData> data = loader.LoadNavData(dir);
  REQUIRE(data);
  CHECK(data.value().cycle == kExpectedCycle);
  // Realistic counts for cycle 2601 (v1): ~256k waypoints, ~95k airway segments,
  // ~17k airports. Allow growth across cycles, but sanity-check the order.
  CHECK(data.value().waypoints.size() > 200000);
  CHECK(data.value().airways.size() > 80000);
  CHECK(data.value().airports.size() > 15000);
  CHECK(!data.value().mora.Empty());
}

TEST_CASE("dfd1: LoadNavData rejects a v2 database with an actionable error",
          "[integration][dfd]") {
  const std::string v2_dir = EnsureDfd2();
  if (v2_dir.empty()) {
    SKIP("DFD v2 data not found");
  }
  bf::Dfd1Loader loader;
  bf::Result<bf::NavData> data = loader.LoadNavData(v2_dir);
  REQUIRE_FALSE(data);
  // The version check fails before any table query; the message points at dfd2.
  CHECK(data.error().code == bf::ErrorCode::kInvalidArgument);
  CHECK(data.error().message.find("dfd2") != std::string::npos);
}

TEST_CASE("dfd1: LoadProcedures returns per-airport procedures", "[integration][dfd]") {
  const std::string dir = EnsureDfd1();
  if (dir.empty()) {
    SKIP("DFD v1 data not found");
  }
  bf::Dfd1Loader loader;
  bf::Result<std::vector<bf::AirportProcedureData>> procs = loader.LoadProcedures(dir);
  REQUIRE(procs);
  CHECK(procs.value().size() > 10000);  // ~17k airports with procedures in 2601
  // KJFK must be present with SID/STAR/approach procedures and runways.
  auto it = std::find_if(procs.value().begin(), procs.value().end(),
                         [](const bf::AirportProcedureData& ap) { return ap.first == "KJFK"; });
  REQUIRE(it != procs.value().end());
  CHECK(!it->second.procedures.empty());
  CHECK(!it->second.runways.empty());
}

TEST_CASE("dfd1: LoadProcedure loads a single airport on demand", "[integration][dfd]") {
  const std::string dir = EnsureDfd1();
  if (dir.empty()) {
    SKIP("DFD v1 data not found");
  }
  bf::Dfd1Loader loader;
  std::optional<bf::CifpData> kjfk = loader.LoadProcedure(dir, "KJFK");
  REQUIRE(kjfk.has_value());
  CHECK(!kjfk->procedures.empty());
  CHECK(!kjfk->runways.empty());
  // An airport with no procedures returns nullopt (pick an unlikely code).
  CHECK_FALSE(loader.LoadProcedure(dir, "ZZZZ").has_value());
}

// ---------------------------------------------------------------------------
// DFD v2 (Dfd2Loader)
// ---------------------------------------------------------------------------

TEST_CASE("dfd2: registry resolves and names itself", "[integration][dfd]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("dfd2");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "dfd2");
}

TEST_CASE("dfd2: LoadNavData parses the enroute dataset", "[integration][dfd]") {
  const std::string dir = EnsureDfd2();
  if (dir.empty()) {
    SKIP("DFD v2 data not found (set up navdata_dfd2/ with a .s3db)");
  }
  bf::Dfd2Loader loader;
  bf::Result<bf::NavData> data = loader.LoadNavData(dir);
  REQUIRE(data);
  CHECK(data.value().cycle == kExpectedCycle);
  // v2 has fewer airports/airways than v1 but the same enroute waypoint count.
  CHECK(data.value().waypoints.size() > 200000);
  CHECK(data.value().airports.size() > 12000);
  CHECK(!data.value().mora.Empty());
}

TEST_CASE("dfd2: LoadNavData rejects a v1 database with an actionable error",
          "[integration][dfd]") {
  const std::string v1_dir = EnsureDfd1();
  if (v1_dir.empty()) {
    SKIP("DFD v1 data not found");
  }
  bf::Dfd2Loader loader;
  bf::Result<bf::NavData> data = loader.LoadNavData(v1_dir);
  REQUIRE_FALSE(data);
  CHECK(data.error().code == bf::ErrorCode::kInvalidArgument);
  CHECK(data.error().message.find("dfd1") != std::string::npos);
}

TEST_CASE("dfd2: LoadProcedure loads a single airport on demand", "[integration][dfd]") {
  const std::string dir = EnsureDfd2();
  if (dir.empty()) {
    SKIP("DFD v2 data not found");
  }
  bf::Dfd2Loader loader;
  std::optional<bf::CifpData> kjfk = loader.LoadProcedure(dir, "KJFK");
  REQUIRE(kjfk.has_value());
  CHECK(!kjfk->procedures.empty());
  CHECK(!kjfk->runways.empty());
}

// ---------------------------------------------------------------------------
// Cross-loader consistency: the same fix must resolve to the same coordinate
// across dfd1, dfd2, and xplane12 (the plan's AROKE KJFK K6 verification point).
// ---------------------------------------------------------------------------

TEST_CASE("dfd: AROKE fix coordinate matches across loaders", "[integration][dfd]") {
  // DFD v1 and v2 both carry AROKE; the query command verified 40.4724, -73.9021.
  // Here we confirm the loaders parse it identically by checking the waypoint
  // exists in each dataset (coordinate equality is exercised via `bf query`).
  const std::string d1 = EnsureDfd1();
  const std::string d2 = EnsureDfd2();
  if (d1.empty() || d2.empty()) {
    SKIP("DFD data not found");
  }
  bf::Dfd1Loader l1;
  bf::Result<bf::NavData> n1 = l1.LoadNavData(d1);
  REQUIRE(n1);
  bf::Dfd2Loader l2;
  bf::Result<bf::NavData> n2 = l2.LoadNavData(d2);
  REQUIRE(n2);
  // AROKE is in region K6; find it in both datasets and compare coordinates.
  auto find = [](const std::vector<bf::Waypoint>& wps) -> const bf::Waypoint* {
    for (const bf::Waypoint& w : wps) {
      if (w.ident.ident == "AROKE") {
        return &w;
      }
    }
    return nullptr;
  };
  const bf::Waypoint* a1 = find(n1.value().waypoints);
  const bf::Waypoint* a2 = find(n2.value().waypoints);
  REQUIRE(a1 != nullptr);
  REQUIRE(a2 != nullptr);
  // Same AIRAC cycle / source data; v1 stores DOUBLE(9) and v2 Float, so the
  // stored precision differs slightly. Coordinates must agree to ~1e-6 deg.
  CHECK(std::fabs(a1->coord.latitude - a2->coord.latitude) < 1e-6);
  CHECK(std::fabs(a1->coord.longitude - a2->coord.longitude) < 1e-6);
}

TEST_CASE("dfd/xplane: KJFK procedure legs agree on rnp/turn/speed across loaders",
          "[integration][dfd]") {
  // The three leg fields (rnp/turn/speed) derive from the same underlying ARINC
  // source, so a leg identified the same way (procedure name + transition + fix +
  // path terminator) must carry identical values whether parsed from the X-Plane
  // CIFP file or the DFD v1 SQLite database. This is both the cross-loader
  // consistency net and the check that the X-Plane column mapping is correct.
  const std::string xp = EnsureXPlane12();  // captured before EnsureDfd1 overwrites env
  const std::string d1 = EnsureDfd1();
  if (d1.empty() || !HasCifp(xp, "KJFK")) {
    SKIP("xplane12 CIFP and/or DFD v1 data not found");
  }

  bf::Result<bf::CifpData> xplane = bf::CifpParser::Parse(xp + "/CIFP/KJFK.dat");
  REQUIRE(xplane);
  bf::Dfd1Loader loader;
  std::optional<bf::CifpData> dfd = loader.LoadProcedure(d1, "KJFK");
  REQUIRE(dfd.has_value());

  struct Extras {
    uint16_t rnp;
    char turn;
    uint16_t speed;
  };
  // Key a leg by name|transition|fix|path_term. Keys occurring more than once in
  // one dataset are ambiguous (can't line up which is which), so drop them.
  auto index = [](const bf::CifpData& c) {
    std::unordered_map<std::string, Extras> unique;
    std::unordered_map<std::string, int> counts;
    for (const bf::Procedure& p : c.procedures) {
      for (const bf::ProcedureLeg& leg : p.legs) {
        std::string key = p.name + "|" + p.transition_ident + "|" +
                          std::string(leg.fix.IdentView()) + "|" +
                          bf::PathTerminatorName(leg.path_term);
        if (++counts[key] == 1) {
          unique[key] = Extras{leg.rnp_centinm, leg.turn_dir, leg.speed_limit_kt};
        } else {
          unique.erase(key);
        }
      }
    }
    return unique;
  };
  const std::unordered_map<std::string, Extras> xa = index(xplane.value());
  const std::unordered_map<std::string, Extras> db = index(*dfd);

  int matched = 0;
  int with_rnp = 0;
  int with_speed = 0;
  for (const auto& [key, xe] : xa) {
    auto it = db.find(key);
    if (it == db.end()) {
      continue;
    }
    ++matched;
    CHECK(xe.rnp == it->second.rnp);
    CHECK(xe.turn == it->second.turn);
    CHECK(xe.speed == it->second.speed);
    if (xe.rnp > 0) {
      ++with_rnp;
    }
    if (xe.speed > 0) {
      ++with_speed;
    }
  }
  // KJFK must share many legs, and the new fields must actually populate (not a
  // vacuous all-zero match), so require some non-zero rnp and speed among them.
  CHECK(matched > 20);
  CHECK(with_rnp > 0);
  CHECK(with_speed > 0);
}
