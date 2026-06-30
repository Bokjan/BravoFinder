#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace {

// Resolve the navigation data directory: BRAVOFINDER_NAVDATA if set, else the
// repository's navdata/ folder. Real Navigraph/Jeppesen data is not committed,
// so these tests SKIP (rather than fail) when the data is absent.
std::string NavDataDir() {
  if (const char* env = std::getenv("BRAVOFINDER_NAVDATA")) {
    return env;
  }
  return "navdata";
}

bool HasData(const std::string& dir) {
  std::ifstream f(dir + "/earth_fix.dat");
  return f.is_open();
}

bool HasCifp(const std::string& dir, const std::string& icao) {
  std::ifstream f(dir + "/CIFP/" + icao + ".dat");
  return f.is_open();
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

TEST_CASE("real data: KJFK to KLAX route is plausible", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);

  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(MakeRequest("KJFK", "KLAX"));
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
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);
  CHECK(db.value().FindRoutes(MakeRequest("kjfk", "klax")));
}

TEST_CASE("real data: unknown airport reports an error", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(MakeRequest("ZZZZ", "KLAX"));
  REQUIRE_FALSE(routes);
  CHECK(routes.error().code == bf::ErrorCode::kAirportNotFound);
}

TEST_CASE("real data: K-shortest returns distinct ordered routes", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.k = 3;
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(req);
  REQUIRE(routes);
  const std::vector<bf::Route>& rs = routes.value();
  REQUIRE(rs.size() >= 2);
  // Routes are ordered shortest-first and must be distinct.
  for (size_t i = 1; i < rs.size(); ++i) {
    CHECK(rs[i].total_distance_nm >= rs[i - 1].total_distance_nm - 1e-6);
    CHECK(rs[i].route_string != rs[i - 1].route_string);
  }
}

TEST_CASE("real data: high cruise altitude still finds a route", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.cruise_fl = 350;  // FL350: enables altitude-band and MORA filtering
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  // The altitude-constrained route should still be a sane length.
  CHECK(routes.value().front().total_distance_nm > 2144.0);
}

TEST_CASE("real data: KJFK to KLAX uses real SID and STAR procedures", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  // This assertion needs the CIFP procedure files, extracted separately from
  // the enroute data; skip if KJFK's/KLAX's procedures are not present.
  if (!HasCifp(dir, "KJFK") || !HasCifp(dir, "KLAX")) {
    SKIP("CIFP procedures not present in '" << dir << "'");
  }

  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(MakeRequest("KJFK", "KLAX"));
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
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  if (!HasCifp(dir, "KJFK") || !HasCifp(dir, "KLAX")) {
    SKIP("CIFP procedures not present in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_runway = "RW31L";  // a real KJFK runway served by DEEZZ5
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK_FALSE(routes.value().front().sid.empty());
}

TEST_CASE("real data: a route does not transit through an intermediate airport", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  if (!HasCifp(dir, "KJFK") || !HasCifp(dir, "KLAX")) {
    SKIP("CIFP procedures not present in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(MakeRequest("KJFK", "KLAX"));
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

TEST_CASE("real data: KJFK publishes terminal-area MSA sectors", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);

  // MSA requires earth_msa.dat, which is extracted separately from the enroute
  // files; skip if it was not provided alongside them.
  const std::vector<bf::MsaSector> msa = db.value().MsaForAirport("KJFK");
  if (msa.empty()) {
    SKIP("earth_msa.dat not present in '" << dir << "'");
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

}  // namespace
