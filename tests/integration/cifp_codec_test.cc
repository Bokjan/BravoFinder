// SPDX-License-Identifier: MIT
#include "io/cache/cifp_codec.h"

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "core/env.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/cache/unified_cache.h"
#include "io/loaders/xplane12/cifp_parser.h"
#include "io/nav_database.h"
#include "test_xplane12.h"

namespace {

using bf::test::EnsureXPlane12;
using bf::test::HasNavData;
using bf::test::NavDataDir;

// Uses the platform temp dir so the test runs on Windows (where /tmp is absent).
std::string TempPath(const std::string& tag) {
  std::filesystem::path dir = std::filesystem::temp_directory_path();
  return (dir / ("bravofinder_cifp_test_" + tag + ".bfdb")).string();
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

// Build a unified .bfdb (graph + CIFP + detail) from real data, returning the
// path (empty on SKIP). The CIFP section is included.
std::string BuildUnified(const std::string& tag) {
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(EnsureXPlane12());
  if (!db) {
    return {};
  }
  const std::string path = TempPath(tag);
  bf::Result<uint32_t> n = db.value().WriteUnified(path);
  REQUIRE(n);
  REQUIRE(n.value() > 0);  // some airports carry procedures
  return path;
}

}  // namespace

TEST_CASE("cifp section: a fetched segment matches direct file parsing", "[integration][cifp]") {
  if (!HasNavData(EnsureXPlane12())) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string path = BuildUnified("seg");
  REQUIRE_FALSE(path.empty());

  bf::Result<bf::UnifiedData> u = bf::UnifiedCache::Open(path);
  REQUIRE(u);
  REQUIRE(u.value().cifp.has_value());

  // KJFK direct parse vs section fetch: procedures and legs must match.
  bf::Result<bf::CifpData> direct = bf::CifpParser::Parse(EnsureXPlane12() + "/CIFP/KJFK.dat");
  REQUIRE(direct);
  auto fetched = u.value().cifp->Fetch("KJFK");
  REQUIRE(fetched.has_value());

  const bf::CifpData& a = direct.value();
  const bf::CifpData& b = fetched.value();
  REQUIRE(a.procedures.size() == b.procedures.size());
  REQUIRE(a.runways.size() == b.runways.size());
  for (size_t i = 0; i < a.procedures.size(); ++i) {
    CHECK(a.procedures[i].name == b.procedures[i].name);
    CHECK(a.procedures[i].transition_ident == b.procedures[i].transition_ident);
    CHECK(a.procedures[i].type == b.procedures[i].type);
    REQUIRE(a.procedures[i].legs.size() == b.procedures[i].legs.size());
    for (size_t j = 0; j < a.procedures[i].legs.size(); ++j) {
      const bf::ProcedureLeg& la = a.procedures[i].legs[j];
      const bf::ProcedureLeg& lb = b.procedures[i].legs[j];
      CHECK(la.fix == lb.fix);
      CHECK(la.path_term == lb.path_term);
      CHECK(lb.course_deg == Catch::Approx(la.course_deg));
      CHECK(lb.distance_nm == Catch::Approx(la.distance_nm));
      CHECK(la.rnp_centinm == lb.rnp_centinm);
      CHECK(la.turn_dir == lb.turn_dir);
      CHECK(la.speed_limit_kt == lb.speed_limit_kt);
    }
  }
  std::remove(path.c_str());
}

TEST_CASE("cifp section: a route via the cache matches the file-based route",
          "[integration][cifp]") {
  const std::string path = BuildUnified("route");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // File-based (data dir with CIFP files) vs cache-based (unified .bfdb).
  bf::Result<bf::NavDatabase> file_db = bf::NavDatabase::Open(EnsureXPlane12());
  bf::Result<bf::NavDatabase> cache_db = bf::NavDatabase::OpenCached(path);
  REQUIRE(file_db);
  REQUIRE(cache_db);

  const bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  bf::Result<std::vector<bf::Route>> a = file_db.value().FindRoutes(req);
  bf::Result<std::vector<bf::Route>> b = cache_db.value().FindRoutes(req);
  REQUIRE(a);
  REQUIRE(b);
  REQUIRE_FALSE(a.value().empty());
  REQUIRE_FALSE(b.value().empty());
  // Same procedures selected from cache as from files.
  CHECK(a.value().front().sid == b.value().front().sid);
  CHECK(a.value().front().star == b.value().front().star);
  CHECK(a.value().front().route_string == b.value().front().route_string);

  std::remove(path.c_str());
}

TEST_CASE("cifp section: procedures resolve from the unified file", "[integration][cifp]") {
  const std::string path = BuildUnified("resolve");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // Procedures must come from the unified file's CIFP section (no source .dat
  // files are read on the cached path).
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::OpenCached(path);
  REQUIRE(db);
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK_FALSE(routes.value().front().sid.empty());  // SID resolved from the cache

  std::remove(path.c_str());
}

TEST_CASE("cifp section: eager loading yields the same route as on-demand", "[integration][cifp]") {
  const std::string path = BuildUnified("eager");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::Result<bf::NavDatabase> on_demand =
      bf::NavDatabase::OpenCached(path, bf::CifpLoad::kOnDemand);
  bf::Result<bf::NavDatabase> eager = bf::NavDatabase::OpenCached(path, bf::CifpLoad::kEager);
  REQUIRE(on_demand);
  REQUIRE(eager);

  const bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  bf::Result<std::vector<bf::Route>> a = on_demand.value().FindRoutes(req);
  bf::Result<std::vector<bf::Route>> b = eager.value().FindRoutes(req);
  REQUIRE(a);
  REQUIRE(b);
  REQUIRE_FALSE(a.value().empty());
  REQUIRE_FALSE(b.value().empty());
  CHECK(a.value().front().route_string == b.value().front().route_string);
  CHECK(a.value().front().sid == b.value().front().sid);
  CHECK(a.value().front().star == b.value().front().star);

  std::remove(path.c_str());
}

TEST_CASE("cifp section: concurrent routing on an eager database is race-free",
          "[integration][cifp]") {
  const std::string path = BuildUnified("eager_concurrent");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::OpenCached(path, bf::CifpLoad::kEager);
  REQUIRE(db);

  // In eager mode ProceduresFor reads the frozen cache without a lock; hammer it
  // from many threads to prove the lock-free read path has no data race (tsan).
  const std::vector<std::pair<std::string, std::string>> pairs = {
      {"KJFK", "KLAX"}, {"KSEA", "KBOS"}, {"KDEN", "KSFO"}, {"KORD", "KDFW"}};
  std::atomic<int> ok{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([&, t]() {
      for (int rep = 0; rep < 10; ++rep) {
        const auto& pr = pairs[(t + rep) % pairs.size()];
        bf::Result<std::vector<bf::Route>> r =
            db.value().FindRoutes(MakeRequest(pr.first, pr.second));
        if (r && !r.value().empty()) {
          ok.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (std::thread& th : threads) {
    th.join();
  }
  CHECK(ok.load() > 0);
  std::remove(path.c_str());
}

TEST_CASE("cifp section: concurrent fetches on one archive are race-free", "[integration][cifp]") {
  const std::string path = BuildUnified("concurrent");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::UnifiedData> u = bf::UnifiedCache::Open(path);
  REQUIRE(u);
  REQUIRE(u.value().cifp.has_value());
  const bf::CifpArchive& archive = u.value().cifp.value();

  // Fetch different airports from many threads at once; Fetch uses a positional
  // read on a shared handle with no mutable cursor, so there is no shared
  // mutable state to race on.
  const std::vector<std::string> icaos = {"KJFK", "KLAX", "KSEA", "KBOS",
                                          "KDEN", "KSFO", "KORD", "KDFW"};
  std::atomic<int> found{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([&, t]() {
      for (int rep = 0; rep < 20; ++rep) {
        auto data = archive.Fetch(icaos[(t + rep) % icaos.size()]);
        if (data.has_value() && !data->procedures.empty()) {
          found.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (std::thread& th : threads) {
    th.join();
  }
  CHECK(found.load() > 0);
  std::remove(path.c_str());
}
