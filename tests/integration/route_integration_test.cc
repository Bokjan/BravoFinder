#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstdlib>
#include <fstream>
#include <string>

#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

using Catch::Matchers::WithinRel;

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

TEST_CASE("real data: KJFK to KLAX route is plausible", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);

  bf::RouteRequest request{"KJFK", "KLAX"};
  bf::Result<bf::Route> route = db.value().FindRoute(request);
  REQUIRE(route);

  const bf::Route& r = route.value();
  // The route must start at the departure and end at the arrival.
  REQUIRE(r.points.size() >= 2);
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KLAX");
  // The great-circle distance JFK-LAX is ~2144 NM. A real airway route follows
  // fixed waypoints, so it is somewhat longer but should stay within ~15%.
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
  CHECK(db.value().FindRoute(bf::RouteRequest{"kjfk", "klax"}));
}

TEST_CASE("real data: unknown airport reports an error", "[integration]") {
  const std::string dir = NavDataDir();
  if (!HasData(dir)) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  REQUIRE(db);
  bf::Result<bf::Route> route = db.value().FindRoute(bf::RouteRequest{"ZZZZ", "KLAX"});
  REQUIRE_FALSE(route);
  CHECK(route.error().code == bf::ErrorCode::kAirportNotFound);
}

}  // namespace
