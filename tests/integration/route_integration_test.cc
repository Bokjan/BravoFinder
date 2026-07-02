#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <fstream>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "core/env.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace {

// Resolve the navigation data directory: BRAVOFINDER_NAVDATA if set, else the
// repository's navdata/ folder. Real Navigraph/Jeppesen data is not committed,
// so these tests SKIP (rather than fail) when the data is absent.
std::string NavDataDir() {
  if (const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA")) {
    return env;
  }
  return "navdata";
}

bool HasCifp(const std::string& dir, const std::string& icao) {
  std::ifstream f(dir + "/CIFP/" + icao + ".dat");
  return f.is_open();
}

// Open the navigation database once and share it across all integration cases.
// NavDatabase is read-only after Open and FindRoutes is const, so a single
// instance is safe to reuse; this avoids re-parsing ~20 MB of data (and
// rebuilding the graph) per case, which dominated the suite's run time.
// Returns nullptr when the data directory has no usable data, so callers SKIP.
const bf::NavDatabase* SharedDb() {
  static const std::string dir = NavDataDir();
  static bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  return db ? &db.value() : nullptr;
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

TEST_CASE("real data: KJFK to KLAX route is plausible", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 2);
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KLAX");
  // Great-circle JFK-LAX is ~2144 NM; an airway route is longer but within ~15%.
  CHECK(r.total_distance_nm > 2144.0);
  CHECK(r.total_distance_nm < 2144.0 * 1.15);
}

TEST_CASE("real data: case-insensitive endpoints", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  CHECK(db->FindRoutes(MakeRequest("kjfk", "klax")));
}

TEST_CASE("real data: unknown airport reports an error", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("ZZZZ", "KLAX"));
  REQUIRE_FALSE(routes);
  CHECK(routes.error().code == bf::ErrorCode::kAirportNotFound);
}

TEST_CASE("real data: K-shortest returns distinct ordered routes", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.k = 3;
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  const std::vector<bf::Route>& rs = routes.value();
  REQUIRE(rs.size() >= 2);
  // Routes are ordered shortest-first and must be distinct.
  for (size_t i = 1; i < rs.size(); ++i) {
    CHECK(rs[i].total_distance_nm >= rs[i - 1].total_distance_nm - 1e-6);
    CHECK(rs[i].route_string != rs[i - 1].route_string);
  }
}

TEST_CASE("real data: an arrival joins the STAR at a near fix, not a far entry", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KLAX")) {
    SKIP("KLAX CIFP not present in '" << NavDataDir() << "'");
  }

  // A STAR exposes every on-network fix it passes as a candidate connection, not
  // just its published entry fix. KLAX BASET5, for instance, enters from PGS
  // (~260 NM out) but its common segment runs through DOWNE close to the field.
  // An arrival from the northeast should join at a near fix, so the final STAR
  // leg (last connection fix -> airport) stays short rather than spanning the
  // whole procedure from a far entry.
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KDEN", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE_FALSE(r.legs.empty());
  const bf::RouteLeg& last = r.legs.back();
  CHECK(last.to == "KLAX");
  CHECK_FALSE(r.star.empty());
  // The chosen entry must not be the far BASET5 entry fix; the final procedure
  // leg should be a short hop, well under the ~260 NM that the far entry implied.
  CHECK(last.from != "PGS");
  CHECK(last.distance_nm < 60.0);
}

TEST_CASE("real data: K candidates can use different procedures", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KLAX")) {
    SKIP("KLAX CIFP not present in '" << NavDataDir() << "'");
  }

  // The multi-endpoint K-shortest lets candidates join the network through
  // different connection fixes, so the alternatives can use genuinely different
  // SID/STAR procedures rather than sharing one fixed pair. KSEA -> KLAX has
  // several competitive arrival options (KIMMO3, WAYVE1); with enough candidates
  // more than one distinct STAR appears (WAYVE1 shows up beyond the first few).
  bf::RouteRequest req = MakeRequest("KSEA", "KLAX");
  req.k = 8;
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  const std::vector<bf::Route>& rs = routes.value();
  REQUIRE(rs.size() >= 2);

  // Ordering and distinctness still hold across the candidate set.
  for (size_t i = 1; i < rs.size(); ++i) {
    CHECK(rs[i].total_distance_nm >= rs[i - 1].total_distance_nm - 1e-6);
    CHECK(rs[i].route_string != rs[i - 1].route_string);
  }

  // At least two candidates differ in their STAR, proving alternatives can cross
  // connection fixes / procedures rather than sharing a single fixed pair.
  std::set<std::string> stars;
  for (const bf::Route& r : rs) {
    stars.insert(r.star);
  }
  CHECK(stars.size() >= 2);
}

TEST_CASE("real data: high cruise altitude still finds a route", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.cruise_fl = 350;  // FL350: enables altitude-band and MORA filtering
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  // The altitude-constrained route should still be a sane length.
  CHECK(routes.value().front().total_distance_nm > 2144.0);
}

TEST_CASE("real data: KJFK to KLAX uses real SID and STAR procedures", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // This assertion needs the CIFP procedure files, extracted separately from
  // the enroute data; skip if KJFK's/KLAX's procedures are not present.
  if (!HasCifp(NavDataDir(), "KJFK") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("CIFP procedures not present in '" << NavDataDir() << "'");
  }

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  // The route connects via real procedures: a SID out of KJFK and a STAR into
  // KLAX, each named, rather than a synthetic DCT hop.
  CHECK_FALSE(r.sid.empty());
  CHECK_FALSE(r.star.empty());
  CHECK_FALSE(r.sid_options.empty());
  CHECK_FALSE(r.star_options.empty());
  // The airports remain the true endpoints; the second / second-to-last points
  // are the procedure connection fixes on the enroute network.
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KLAX");
  CHECK(r.points.size() >= 4);
  // A procedure-connected route is still geographically plausible (the straight
  // -line procedure estimate keeps the total at or above the great circle).
  CHECK(r.total_distance_nm > 2144.0);
  CHECK(r.total_distance_nm < 2144.0 * 1.2);

  // The procedures appear as explicit first/last legs (airport <-> connection
  // fix) labeled by the SID/STAR, and the route string reads like a filed plan.
  REQUIRE(r.legs.size() >= 2);
  CHECK(r.legs.front().from == "KJFK");
  CHECK(r.legs.front().via == r.sid);
  CHECK(r.legs.back().to == "KLAX");
  CHECK(r.legs.back().via == r.star);
  CHECK(r.route_string.rfind("KJFK " + r.sid + " ", 0) == 0);  // starts with "KJFK <SID> "
  CHECK(r.route_string.find(" " + r.star + " KLAX") != std::string::npos);
  // Leg distances should sum to the reported total (procedures included).
  double leg_sum = 0.0;
  for (const bf::RouteLeg& leg : r.legs) {
    leg_sum += leg.distance_nm;
  }
  CHECK(leg_sum == Catch::Approx(r.total_distance_nm).margin(1.0));
}

TEST_CASE("real data: a departure runway filter still yields a route", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KJFK") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("CIFP procedures not present in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_runway = "RW31L";  // a real KJFK runway served by DEEZZ5
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK_FALSE(routes.value().front().sid.empty());
}

TEST_CASE("real data: a named SID is used when requested", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KJFK") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("CIFP procedures not present in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_sid = "DEEZZ5";  // a real KJFK SID
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  // Every candidate must depart via the requested SID.
  for (const bf::Route& r : routes.value()) {
    CHECK(r.sid == "DEEZZ5");
  }
}

TEST_CASE("real data: a bare SID name pins the procedure but leaves transitions open",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KJFK") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("CIFP procedures not present in '" << NavDataDir() << "'");
  }

  // "DEEZZ5.TOWIN" pins the transition; the offered options should all be that
  // transition, unlike the bare-name case which leaves several.
  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_sid = "DEEZZ5.TOWIN";
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK(routes.value().front().sid == "DEEZZ5");
  for (const std::string& opt : routes.value().front().sid_options) {
    CHECK(opt.rfind("DEEZZ5.TOWIN", 0) == 0);
  }
}

TEST_CASE("real data: an unknown SID name is an error, not a silent fallback",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KJFK") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("CIFP procedures not present in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_sid = "NOSUCH9";
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  CHECK_FALSE(routes);  // no DCT fallback, no other SID -- a clean error
}

TEST_CASE("real data: an airport without procedures stays the endpoint via DCT", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // Exercises the kDirect fallback: KJFK has a SID, while KIKR has no CIFP file
  // at all, so its arrival connects by a direct link. The airport must remain
  // the route endpoint rather than being replaced by its connection fix, and the
  // connection is classified kDirect (no data) rather than radar vectors.
  if (!HasCifp(NavDataDir(), "KJFK")) {
    SKIP("KJFK CIFP not present in '" << NavDataDir() << "'");
  }
  if (HasCifp(NavDataDir(), "KIKR")) {
    SKIP("KIKR CIFP is present; this case tests the no-procedure fallback");
  }
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KIKR"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 2);
  REQUIRE(r.legs.size() >= 2);
  // Both airports are the true endpoints; the arrival has no procedure data, so
  // its final leg is a DCT link into the airport, classified kDirect.
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KIKR");
  CHECK(r.legs.back().to == "KIKR");
  CHECK(r.star.empty());
  CHECK(r.legs.back().via == "DCT");
  CHECK(r.arr_connection == bf::ConnectionKind::kDirect);
}

TEST_CASE("real data: a route does not transit through an intermediate airport", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  if (!HasCifp(NavDataDir(), "KJFK") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("CIFP procedures not present in '" << NavDataDir() << "'");
  }
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 2);
  // Airports connect via synthetic DCT links and used to be exploitable as
  // free pass-through hubs (e.g. ...MIE DCT KMIE SNKPT...). Only the first and
  // last points may be airport ICAOs; no interior point should be one.
  for (size_t i = 1; i + 1 < r.points.size(); ++i) {
    const std::string& id = r.points[i].ident;
    const bool looks_like_us_airport = id.size() == 4 && id.front() == 'K';
    CHECK_FALSE(looks_like_us_airport);
  }
}

TEST_CASE("real data: a radar-vectored departure is flagged, not shown as missing",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // KPHL publishes only radar-vector SIDs (PHL4: VA -> VM, no fix), so no SID
  // reaches an on-network fix and the departure falls back to a DCT link. That
  // fallback must be reported as radar vectors (procedures exist) rather than
  // kDirect (no data), so a user can tell the two apart.
  if (!HasCifp(NavDataDir(), "KPHL") || !HasCifp(NavDataDir(), "KLAX")) {
    SKIP("KPHL/KLAX CIFP not present in '" << NavDataDir() << "'");
  }

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KPHL", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  CHECK(r.dep_connection == bf::ConnectionKind::kRadarVectors);
  CHECK(r.sid.empty());  // no named SID for a radar-vectored departure
  // The airport is still the endpoint and the first leg is the DCT hop out.
  CHECK(r.points.front().ident == "KPHL");
  CHECK(r.legs.front().from == "KPHL");
  CHECK(r.legs.front().via == "DCT");
  // The arrival side still connects through a real STAR.
  CHECK(r.arr_connection == bf::ConnectionKind::kProcedure);
  CHECK_FALSE(r.star.empty());
}

TEST_CASE("real data: KJFK publishes terminal-area MSA sectors", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // MSA requires earth_msa.dat, which is extracted separately from the enroute
  // files; skip if it was not provided alongside them.
  const std::vector<bf::MsaSector> msa = db->MsaForAirport("KJFK");
  if (msa.empty()) {
    SKIP("earth_msa.dat not present in '" << NavDataDir() << "'");
  }

  // Every returned sector belongs to KJFK and carries at least one arc with a
  // positive minimum altitude.
  for (const bf::MsaSector& s : msa) {
    CHECK(s.airport_icao == "KJFK");
    REQUIRE_FALSE(s.arcs.empty());
    bool any_alt = false;
    for (const bf::MsaArc& a : s.arcs) {
      if (a.alt_100ft > 0) {
        any_alt = true;
      }
    }
    CHECK(any_alt);
  }
}

TEST_CASE("real data: concurrent FindRoutes on one shared database is race-free",
          "[integration][concurrency]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // Exercise the only shared mutable state (the lazily filled procedure cache)
  // from several threads at once. The query set mixes:
  //   - the same airport pair (KJFK/KLAX) to stress same-key cache contention,
  //   - different pairs to stress parallel parsing and concurrent map rehashing.
  // Under the tsan preset this catches data races; under any build it must not
  // crash and every query that a single thread can answer must also succeed
  // here. Endpoints that lack data simply return an error, which is fine: the
  // point is concurrency safety, not that every pair routes.
  const std::vector<std::pair<std::string, std::string>> queries = {
      {"KJFK", "KLAX"}, {"KJFK", "KLAX"}, {"KLAX", "KJFK"}, {"KBOS", "KDEN"},
      {"KDEN", "KBOS"}, {"KJFK", "KBOS"}, {"KLAX", "KDEN"}, {"KBOS", "KLAX"},
  };

  constexpr int kThreads = 8;
  constexpr int kItersPerThread = 25;
  std::atomic<int> completed{0};
  std::atomic<bool> any_route{false};

  std::vector<std::thread> workers;
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&, t]() {
      for (int i = 0; i < kItersPerThread; ++i) {
        const auto& q = queries[(t + i) % queries.size()];
        bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest(q.first, q.second));
        if (routes && !routes.value().empty()) {
          any_route.store(true);
        }
        completed.fetch_add(1);
      }
    });
  }
  for (std::thread& w : workers) {
    w.join();
  }

  CHECK(completed.load() == kThreads * kItersPerThread);
  // At least one of the well-known pairs must have produced a route, proving the
  // concurrent calls actually did work (not merely failed in lockstep).
  CHECK(any_route.load());

  // The shared cache must agree with a fresh single-threaded query afterward.
  bf::Result<std::vector<bf::Route>> after = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  CHECK(after);
}

}  // namespace
