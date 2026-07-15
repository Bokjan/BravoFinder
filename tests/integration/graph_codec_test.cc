#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/env.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/unified_cache.h"
#include "io/nav_database.h"
#include "test_xplane12.h"

namespace {

using bf::test::EnsureXPlane12;
using bf::test::NavDataDir;

// A unique temp path for a .bfdb produced by a test. Uses the test name so
// parallel cases do not collide. Uses the platform temp dir so the test runs on
// Windows (where /tmp is absent).
std::string TempBfdb(const std::string& tag) {
  std::filesystem::path dir = std::filesystem::temp_directory_path();
  return (dir / ("bravofinder_test_" + tag + ".bfdb")).string();
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

// Build a database and write it to a unified .bfdb, returning the path (empty on
// SKIP). Includes the CIFP section so the cached path resolves procedures.
std::string BuildCache(const std::string& tag) {
  const std::string dir = EnsureXPlane12();
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  if (!db) {
    return {};
  }
  const std::string path = TempBfdb(tag);
  bf::Result<uint32_t> written = db.value().WriteUnified(path);
  REQUIRE(written);
  return path;
}

}  // namespace

TEST_CASE("bfdb: a cached route matches the freshly built route", "[integration][bfdb]") {
  const std::string dir = EnsureXPlane12();
  bf::Result<bf::NavDatabase> direct = bf::NavDatabase::Open(dir);
  if (!direct) {
    SKIP("navigation data not found in '" << dir << "' (set BRAVOFINDER_NAVDATA)");
  }
  const std::string path = TempBfdb("roundtrip");
  // The unified file carries graph + CIFP + detail, so the cached path resolves
  // the same procedures (SID/STAR) the direct path parses from CIFP files.
  REQUIRE(direct.value().WriteUnified(path));

  bf::Result<bf::NavDatabase> cached = bf::NavDatabase::OpenCached(path);
  REQUIRE(cached);

  // Same query on both databases must yield identical routes.
  const bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  bf::Result<std::vector<bf::Route>> a = direct.value().FindRoutes(req);
  bf::Result<std::vector<bf::Route>> b = cached.value().FindRoutes(req);
  REQUIRE(a);
  REQUIRE(b);
  REQUIRE_FALSE(a.value().empty());
  REQUIRE(a.value().size() == b.value().size());

  const bf::Route& ra = a.value().front();
  const bf::Route& rb = b.value().front();
  CHECK(ra.route_string == rb.route_string);
  CHECK(ra.sid == rb.sid);
  CHECK(ra.star == rb.star);
  CHECK(ra.legs.size() == rb.legs.size());
  // Distance stored as float in the cache; allow a small tolerance.
  CHECK(rb.total_distance_nm == Catch::Approx(ra.total_distance_nm).margin(0.01));
  for (size_t i = 0; i < ra.legs.size(); ++i) {
    CHECK(ra.legs[i].from == rb.legs[i].from);
    CHECK(ra.legs[i].to == rb.legs[i].to);
    CHECK(ra.legs[i].via == rb.legs[i].via);
  }

  std::remove(path.c_str());
}

TEST_CASE("bfdb: the container header preserves AIRAC provenance", "[integration][bfdb]") {
  const std::string path = BuildCache("meta");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::UnifiedData> u = bf::UnifiedCache::Open(path);
  REQUIRE(u);
  // cycle 2601 for the test dataset; should be non-zero and the graph non-empty.
  CHECK(u.value().header.cycle != 0);
  CHECK_FALSE(u.value().graph.coords.empty());
  CHECK(u.value().graph.offsets.size() == u.value().graph.coords.size() + 1);
  std::remove(path.c_str());
}

TEST_CASE("bfdb: the cache preserves waypoint kinds and airport elevations",
          "[integration][bfdb]") {
  const std::string path = BuildCache("fields");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::UnifiedData> u = bf::UnifiedCache::Open(path);
  REQUIRE(u);
  const bf::GraphSnapshot& snapshot = u.value().graph;

  // Per-vertex kinds are present for every vertex.
  REQUIRE(snapshot.kinds.size() == snapshot.coords.size());
  // Real AIRAC data has a mix of fixes and navaids, so at least one vertex must
  // be a navaid kind -- proving the field is populated, not defaulted to kFix.
  bool has_navaid = false;
  for (bf::WaypointKind k : snapshot.kinds) {
    if (k == bf::WaypointKind::kVor || k == bf::WaypointKind::kNdb || k == bf::WaypointKind::kDme) {
      has_navaid = true;
      break;
    }
  }
  CHECK(has_navaid);

  // Airport elevations: one per airport vertex, and at least one non-zero (most
  // airports sit above sea level), proving elevation survived the round-trip.
  const size_t airport_count =
      snapshot.coords.size() - static_cast<size_t>(snapshot.first_airport_vertex);
  REQUIRE(snapshot.airport_elevations_ft.size() == airport_count);
  bool has_nonzero_elev = false;
  for (int e : snapshot.airport_elevations_ft) {
    if (e != 0) {
      has_nonzero_elev = true;
      break;
    }
  }
  CHECK(has_nonzero_elev);

  // Per-vertex airway-membership flags: both directions are present for every
  // vertex, and at least one vertex is inbound-only (has_inbound && !has_outbound)
  // -- a STAR entry gate reached only via a forward-only airway (e.g. ABBEY).
  // This proves flags bit1 survived the round-trip and inbound-only terminals
  // are not collapsed to off-network.
  REQUIRE(snapshot.has_outbound.size() == snapshot.coords.size());
  REQUIRE(snapshot.has_inbound.size() == snapshot.coords.size());
  bool has_inbound_only = false;
  for (size_t i = 0; i < snapshot.coords.size(); ++i) {
    if (snapshot.has_inbound[i] && !snapshot.has_outbound[i]) {
      has_inbound_only = true;
      break;
    }
  }
  CHECK(has_inbound_only);

  std::remove(path.c_str());
}

TEST_CASE("bfdb: an airport without procedures still routes via the cache", "[integration][bfdb]") {
  const std::string dir = EnsureXPlane12();
  const std::string path = BuildCache("nocifp");
  if (path.empty()) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> cached = bf::NavDatabase::OpenCached(path);
  REQUIRE(cached);
  // KIKR has no CIFP file; the airport should still connect via DCT fallback.
  bf::Result<std::vector<bf::Route>> routes =
      cached.value().FindRoutes(MakeRequest("KIKR", "KLAX"));
  // Either a route is found (DCT fallback works) or a clean no-route error; the
  // point is that OpenCached does not crash on missing CIFP.
  if (routes) {
    CHECK_FALSE(routes.value().empty());
  } else {
    CHECK(routes.error().code != bf::ErrorCode::kUnknown);
  }
  std::remove(path.c_str());
}

TEST_CASE("bfdb: a corrupt or missing cache is rejected cleanly", "[unit][bfdb]") {
  // Missing file.
  {
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(TempBfdb("does_not_exist"));
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
  }
  // Bad magic.
  {
    const std::string path = TempBfdb("badmagic");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "NOPEnot a real bfdb file at all";
    f.close();
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
  // Right magic, wrong version.
  {
    const std::string path = TempBfdb("badver");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "BFDB";
    const uint32_t bad_version = 0xDEADBEEF;
    f.write(reinterpret_cast<const char*>(&bad_version), sizeof(bad_version));
    f.close();
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kFormatMismatch);
    std::remove(path.c_str());
  }
  // Truncated (only magic).
  {
    const std::string path = TempBfdb("trunc");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "BF";
    f.close();
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
}
