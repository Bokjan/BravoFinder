#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "io/nav_database.h"
#include "test_bfdb.h"

namespace {

using bf::test::NavDataDir;

// Share one opened database across cases (read-only after Open; lookups are
// const). Prefers a prebuilt cache for fast startup. Returns nullptr when data
// is absent so callers SKIP.
const bf::NavDatabase* SharedDb() {
  static bf::Result<bf::NavDatabase> db = bf::test::OpenReadOnlyDb();
  return db ? &db.value() : nullptr;
}

}  // namespace

TEST_CASE("query: batch waypoint lookup is parallel to input with kinds", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // A known fix, a known VOR, and a bogus ident: results must be parallel to the
  // input, with an empty group for the miss. Each group holds every region
  // match for that ident.
  const std::vector<std::string> ids{"NINOX", "DGC", "ZZ_NOT_REAL_ZZ"};
  auto r = db->LookupWaypoints(ids);
  REQUIRE(r.size() == ids.size());
  REQUIRE_FALSE(r[0].empty());
  CHECK(r[0][0].ident == "NINOX");
  CHECK(r[0][0].kind == bf::WaypointKind::kFix);
  REQUIRE_FALSE(r[1].empty());
  CHECK(r[1][0].ident == "DGC");
  CHECK(r[1][0].kind == bf::WaypointKind::kVor);  // DGC is a VOR
  CHECK(r[1][0].on_network);
  CHECK(r[2].empty());
}

TEST_CASE("query: case-insensitive waypoint ident", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  auto r = db->LookupWaypoints({"ninox"});
  REQUIRE(r.size() == 1);
  REQUIRE_FALSE(r[0].empty());
  CHECK(r[0][0].ident == "NINOX");
}

TEST_CASE("query: airport lookup carries elevation and procedure flag", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  auto r = db->LookupAirports({"KJFK", "KIKR", "NOPE"});
  REQUIRE(r.size() == 3);
  REQUIRE(r[0].has_value());
  CHECK(r[0]->icao == "KJFK");
  CHECK(r[0]->coord.latitude > 40.0);
  CHECK(r[0]->coord.latitude < 41.0);
  CHECK(r[0]->has_procedures);  // KJFK publishes CIFP procedures
  REQUIRE(r[1].has_value());
  CHECK(r[1]->icao == "KIKR");
  CHECK_FALSE(r[1]->has_procedures);  // KIKR has no CIFP file
  CHECK(r[1]->elevation_ft > 0);      // KIKR sits well above sea level
  CHECK_FALSE(r[2].has_value());
}

TEST_CASE("query: procedure lookup lists SIDs for an airport with CIFP", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  auto r = db->LookupProcedures({"KJFK", "KIKR"});
  REQUIRE(r.size() == 2);
  REQUIRE(r[0].has_value());
  CHECK(r[0]->icao == "KJFK");
  CHECK_FALSE(r[0]->procedures.empty());
  bool has_sid = false;
  for (const bf::ProcedureSummary& p : r[0]->procedures) {
    if (p.type == bf::ProcedureType::kSid) {
      has_sid = true;
      break;
    }
  }
  CHECK(has_sid);
  CHECK_FALSE(r[1].has_value());  // KIKR has no CIFP
}

TEST_CASE("query: procedure detail surfaces per-leg rnp/turn/speed", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // KJFK R13L is an RNAV approach: its RF leg to JEVNI carries RNP 0.30 and a
  // right turn, and it publishes a speed limit on an early leg. This exercises
  // the three new leg fields end to end through the query path.
  std::optional<bf::AirportProcedureDetail> d = db->LookupProcedureDetail("kjfk", "R13L");
  REQUIRE(d.has_value());
  CHECK(d->icao == "KJFK");
  CHECK(d->procedure == "R13L");
  REQUIRE_FALSE(d->transitions.empty());

  bool saw_rf_rnp_turn = false;
  bool saw_speed = false;
  for (const bf::ProcedureDetail& t : d->transitions) {
    for (const bf::ProcedureLegInfo& leg : t.legs) {
      if (leg.fix == "JEVNI" && leg.path_term == "RF") {
        CHECK(leg.rnp_nm == Catch::Approx(0.30));
        CHECK(leg.turn_dir == 'R');
        saw_rf_rnp_turn = true;
      }
      if (leg.speed_limit_kt > 0) {
        saw_speed = true;
      }
    }
  }
  CHECK(saw_rf_rnp_turn);
  CHECK(saw_speed);

  // An airport without CIFP, and an unknown procedure name, both yield nullopt.
  CHECK_FALSE(db->LookupProcedureDetail("KIKR", "ANYTHING").has_value());
  CHECK_FALSE(db->LookupProcedureDetail("KJFK", "NO_SUCH_PROC").has_value());
}

TEST_CASE("query: airway lookup returns directed segments", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  auto r = db->LookupAirways({"Y28", "ZZ_NOT_AN_AIRWAY"});
  REQUIRE(r.size() == 2);
  REQUIRE(r[0].has_value());
  CHECK(r[0]->name == "Y28");
  CHECK_FALSE(r[0]->segments.empty());
  for (const bf::AirwayLeg& s : r[0]->segments) {
    CHECK_FALSE(s.from.empty());
    CHECK_FALSE(s.to.empty());
    CHECK(s.distance_nm > 0.0);
  }
  CHECK_FALSE(r[1].has_value());
}

TEST_CASE("query: concurrent airway is found by each of its designators", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // "A1-G581" is a real concurrency in AIRAC data: one physical segment carrying
  // both A1 and G581. Before the designator-split fix, LookupAirways("A1") missed
  // it (the index was keyed on the whole "A1-G581" string). Both designators must
  // now resolve, and must share at least one identical physical segment.
  auto by_a1 = db->LookupAirways({"A1"});
  auto by_g581 = db->LookupAirways({"G581"});
  if (!by_a1[0].has_value() || !by_g581[0].has_value()) {
    SKIP("expected concurrency A1-G581 not present in this AIRAC cycle");
  }
  auto same_leg = [](const bf::AirwayLeg& x, const bf::AirwayLeg& y) {
    return x.from == y.from && x.to == y.to && std::abs(x.distance_nm - y.distance_nm) < 1e-3;
  };
  bool shared = false;
  for (const bf::AirwayLeg& a : by_a1[0]->segments) {
    for (const bf::AirwayLeg& g : by_g581[0]->segments) {
      if (same_leg(a, g)) {
        shared = true;
        break;
      }
    }
    if (shared) {
      break;
    }
  }
  CHECK(shared);
}

TEST_CASE("query: navaid detail lookup carries freq/range/elevation", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // Navaid detail comes from the unified file's detail section (cache path) or
  // is built in-memory (raw-data path). Either way DGC (a VOR) must carry sane
  // attributes.
  auto r = db->LookupNavaidDetails({"DGC", "ZZ_NOT_REAL_ZZ"});
  REQUIRE(r.size() == 2);
  if (r[0].empty()) {
    SKIP("no navaid detail available (the .bfdb has no detail section)");
  }
  const bf::NavaidDetailInfo& d = r[0][0];
  CHECK(d.ident == "DGC");
  CHECK(d.kind == bf::WaypointKind::kVor);
  CHECK(d.freq_raw > 10000);  // a VOR frequency, MHz*100 (e.g. 11500 = 115.0)
  CHECK(d.freq_raw < 12000);
  CHECK(d.range_nm > 0.0);
  CHECK(r[1].empty());  // the bogus ident matches nothing
}

TEST_CASE("query: hold lookup returns holding-pattern parameters", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // AE701 (region DA, airport DAAE) is a real enroute-terminal hold in cycle
  // 2601: inbound 171, 1.0 min leg, right turns, 5580-14000 ft, 230 kt.
  auto r = db->LookupHolds({"AE701", "ZZ_NOT_REAL_ZZ"});
  REQUIRE(r.size() == 2);
  if (r[0].empty()) {
    SKIP("no hold data available (needs earth_hold.dat parsed into a detail cache)");
  }
  const bf::HoldInfo& h = r[0][0];
  CHECK(h.fix_ident == "AE701");
  CHECK(h.inbound_course > 0.0);
  CHECK((h.turn_dir == 'R' || h.turn_dir == 'L'));
  CHECK(h.min_alt_ft > 0);
  CHECK(r[1].empty());
}

TEST_CASE("query: concurrent lookups on one database are race-free", "[integration][query]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // The airway index is built once at Open and then read-only; procedures use
  // the internally synchronized cache. Hammer all four lookups from several
  // threads to exercise contract B under tsan.
  std::atomic<int> ok{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([&] {
      for (int i = 0; i < 20; ++i) {
        auto w = db->LookupWaypoints({"NINOX", "DGC"});
        auto a = db->LookupAirports({"KJFK"});
        auto p = db->LookupProcedures({"KJFK"});
        auto ai = db->LookupAirways({"Y28"});
        auto nd = db->LookupNavaidDetails({"DGC"});
        auto h = db->LookupHolds({"AE701"});
        if (!w[0].empty() && !w[1].empty() && a[0] && p[0] && ai[0] && nd.size() == 1 &&
            h.size() == 1) {
          ok.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (std::thread& th : threads) {
    th.join();
  }
  CHECK(ok.load() == 8 * 20);
}
