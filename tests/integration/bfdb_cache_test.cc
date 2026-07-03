#include "io/cache/bfdb_cache.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "core/env.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace {

// Resolve the navigation data directory: BRAVOFINDER_NAVDATA if set, else the
// repository's navdata/ folder. Real data is not committed, so these tests SKIP
// (rather than fail) when it is absent.
std::string NavDataDir() {
  if (const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA")) {
    return env;
  }
  return "navdata";
}

// A unique temp path for a .bfdb produced by a test. Uses the test name so
// parallel cases do not collide.
std::string TempBfdb(const std::string& tag) {
  return std::string("/tmp/bravofinder_test_") + tag + ".bfdb";
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

// Build a database, write it to a .bfdb, and return the path (or empty on SKIP).
std::string BuildCache(const std::string& tag) {
  const std::string dir = NavDataDir();
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  if (!db) {
    return {};
  }
  const std::string path = TempBfdb(tag);
  bf::Result<void> written = db.value().WriteCache(path);
  REQUIRE(written);
  return path;
}

}  // namespace

TEST_CASE("bfdb: a cached route matches the freshly built route", "[integration][bfdb]") {
  const std::string dir = NavDataDir();
  bf::Result<bf::NavDatabase> direct = bf::NavDatabase::Open(dir);
  if (!direct) {
    SKIP("navigation data not found in '" << dir << "' (set BRAVOFINDER_NAVDATA)");
  }
  const std::string path = TempBfdb("roundtrip");
  REQUIRE(direct.value().WriteCache(path));
  // Also write the sibling <stem>_cifp.bfdb so the cached path resolves the same
  // procedures (SID/STAR) the direct path parses from CIFP files.
  const std::string cifp_path = "/tmp/bravofinder_test_roundtrip_cifp.bfdb";
  REQUIRE(direct.value().WriteCifpCache(cifp_path));

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
  std::remove(cifp_path.c_str());
}

TEST_CASE("bfdb: the cache header preserves AIRAC provenance", "[integration][bfdb]") {
  const std::string path = BuildCache("meta");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::GraphArchive> arc = bf::BfdbCache::Read(path);
  REQUIRE(arc);
  // cycle 2601 / build 20260112 for the test dataset; both should be non-zero
  // and the graph non-empty.
  CHECK(arc.value().cycle != 0);
  CHECK(arc.value().build != 0);
  CHECK_FALSE(arc.value().coords.empty());
  CHECK(arc.value().offsets.size() == arc.value().coords.size() + 1);
  std::remove(path.c_str());
}

TEST_CASE("bfdb: the cache preserves waypoint kinds and airport elevations",
          "[integration][bfdb]") {
  const std::string path = BuildCache("fields");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::GraphArchive> arc = bf::BfdbCache::Read(path);
  REQUIRE(arc);
  const bf::GraphArchive& arcv = arc.value();

  // Per-vertex kinds are present for every vertex.
  REQUIRE(arcv.kinds.size() == arcv.coords.size());
  // Real AIRAC data has a mix of fixes and navaids, so at least one vertex must
  // be a navaid kind -- proving the field is populated, not defaulted to kFix.
  bool has_navaid = false;
  for (bf::WaypointKind k : arcv.kinds) {
    if (k == bf::WaypointKind::kVor || k == bf::WaypointKind::kNdb || k == bf::WaypointKind::kDme) {
      has_navaid = true;
      break;
    }
  }
  CHECK(has_navaid);

  // Airport elevations: one per airport vertex, and at least one non-zero (most
  // airports sit above sea level), proving elevation survived the round-trip.
  const size_t airport_count = arcv.coords.size() - static_cast<size_t>(arcv.first_airport_vertex);
  REQUIRE(arcv.airport_elevations_ft.size() == airport_count);
  bool has_nonzero_elev = false;
  for (int e : arcv.airport_elevations_ft) {
    if (e != 0) {
      has_nonzero_elev = true;
      break;
    }
  }
  CHECK(has_nonzero_elev);

  std::remove(path.c_str());
}

TEST_CASE("bfdb: an airport without procedures still routes via the cache", "[integration][bfdb]") {
  const std::string dir = NavDataDir();
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
    bf::Result<bf::GraphArchive> r = bf::BfdbCache::Read("/tmp/bravofinder_does_not_exist.bfdb");
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
  }
  // Bad magic.
  {
    const std::string path = TempBfdb("badmagic");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "NOPEnot a real bfdb file at all";
    f.close();
    bf::Result<bf::GraphArchive> r = bf::BfdbCache::Read(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
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
    bf::Result<bf::GraphArchive> r = bf::BfdbCache::Read(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
    std::remove(path.c_str());
  }
  // Truncated (only magic).
  {
    const std::string path = TempBfdb("trunc");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "BF";
    f.close();
    bf::Result<bf::GraphArchive> r = bf::BfdbCache::Read(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
    std::remove(path.c_str());
  }
  // Valid magic and version, but an absurd vertex count in the header. The
  // counts must be range-checked against the file size BEFORE any resize, or a
  // forged header would trigger a huge allocation and crash instead of a clean
  // Result. (Regression guard for the header-count bounds check.)
  {
    const std::string path = TempBfdb("boguscount");
    auto put_u32 = [](std::string& s, uint32_t v) {
      for (int i = 0; i < 4; ++i) {
        s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
      }
    };
    std::string buf;
    buf.append("BFDB", 4);
    put_u32(buf, bf::BfdbCache::kFormatVersion);
    put_u32(buf, 2601);        // cycle
    put_u32(buf, 20260112);    // build
    put_u32(buf, 0xFFFFFFFF);  // v: absurd vertex count
    put_u32(buf, 0);           // e
    put_u32(buf, 0);           // airway_count
    put_u32(buf, 0);           // msa_count
    put_u32(buf, 0);           // first_airport_vertex
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(buf.data(), static_cast<std::streamsize>(buf.size()));
    f.close();
    bf::Result<bf::GraphArchive> r = bf::BfdbCache::Read(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
    std::remove(path.c_str());
  }
}
