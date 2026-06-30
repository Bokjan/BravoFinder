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

}  // namespace
