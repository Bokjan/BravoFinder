#include "io/cache/cifp_cache.h"

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "core/env.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "core/version.h"
#include "io/loaders/xplane12/cifp/cifp_parser.h"
#include "io/loaders/xplane12/xplane12_loader.h"
#include "io/nav_database.h"

namespace {

std::string NavDataDir() {
  if (const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA")) {
    return env;
  }
  return "navdata";
}

bool HasNavData() {
  std::ifstream f(NavDataDir() + "/earth_fix.dat");
  return f.is_open();
}

std::string TempPath(const std::string& tag) {
  return std::string("/tmp/bravofinder_cifp_test_") + tag;
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

// Parse the full CIFP set from real data and write it to `cifp_path`, mirroring
// what NavDatabase::WriteCifpCache does. Returns the airport count Result.
bf::Result<uint32_t> BuildCifpCache(const std::string& cifp_path) {
  bf::XPlane12Loader loader;
  bf::Result<std::vector<bf::AirportProcedureData>> procs = loader.LoadProcedures(NavDataDir());
  if (!procs) {
    return bf::Result<uint32_t>::Err(std::move(procs).error());
  }
  return bf::CifpCache::Build(procs.value(), cifp_path, "xplane12", 2601, 20260112,
                              bf::kBravoFinderVersion);
}

// Build both caches (graph + CIFP) from real data into a temp dir, returning the
// graph cache path (empty on SKIP). The CIFP cache is the sibling *_cifp.bfdb,
// so OpenCached(graph_path) auto-discovers it.
std::string BuildBothCaches(const std::string& tag) {
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(NavDataDir());
  if (!db) {
    return {};
  }
  const std::string graph_path = TempPath(tag + "_nav.bfdb");
  const std::string cifp_path = TempPath(tag + "_nav_cifp.bfdb");
  REQUIRE(db.value().WriteCache(graph_path));
  bf::Result<uint32_t> n = db.value().WriteCifpCache(cifp_path);
  REQUIRE(n);
  REQUIRE(n.value() > 0);
  return graph_path;
}

}  // namespace

TEST_CASE("cifp cache: a fetched segment matches direct file parsing", "[integration][cifp]") {
  if (!HasNavData()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string cifp_path = TempPath("seg_nav_cifp.bfdb");
  bf::Result<uint32_t> n = BuildCifpCache(cifp_path);
  REQUIRE(n);

  bf::Result<bf::CifpArchive> archive = bf::CifpCache::Open(cifp_path);
  REQUIRE(archive);
  CHECK(archive.value().source_loader() == "xplane12");
  CHECK(archive.value().program_semver() == bf::kBravoFinderVersion);
  CHECK(archive.value().cycle() == 2601);

  // KJFK direct parse vs archive fetch: procedures and legs must match.
  bf::Result<bf::CifpData> direct = bf::CifpParser::Parse(NavDataDir() + "/CIFP/KJFK.dat");
  REQUIRE(direct);
  auto fetched = archive.value().Fetch("KJFK");
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
      CHECK(la.fix.ident == lb.fix.ident);
      CHECK(la.fix.region == lb.fix.region);
      CHECK(la.path_term == lb.path_term);
      CHECK(lb.course_deg == Catch::Approx(la.course_deg));
      CHECK(lb.distance_nm == Catch::Approx(la.distance_nm));
    }
  }
  std::remove(cifp_path.c_str());
}

TEST_CASE("cifp cache: a route via the cache matches the file-based route", "[integration][cifp]") {
  const std::string graph_path = BuildBothCaches("route");
  if (graph_path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string cifp_path = TempPath("route_nav_cifp.bfdb");

  // File-based (data dir with CIFP files) vs cache-based (sibling _cifp.bfdb
  // auto-discovered next to the graph cache).
  bf::Result<bf::NavDatabase> file_db = bf::NavDatabase::Open(NavDataDir());
  bf::Result<bf::NavDatabase> cache_db = bf::NavDatabase::OpenCached(graph_path);
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

  std::remove(graph_path.c_str());
  std::remove(cifp_path.c_str());
}

TEST_CASE("cifp cache: a sibling _cifp.bfdb is auto-discovered", "[integration][cifp]") {
  const std::string graph_path = BuildBothCaches("auto");
  if (graph_path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string cifp_path = TempPath("auto_nav_cifp.bfdb");
  REQUIRE(std::filesystem::exists(cifp_path));

  // Procedures must come from the auto-discovered sibling cache (no source
  // .dat files are read on the cached path).
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::OpenCached(graph_path);
  REQUIRE(db);
  bf::Result<std::vector<bf::Route>> routes = db.value().FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK_FALSE(routes.value().front().sid.empty());  // SID resolved from the cache

  std::remove(graph_path.c_str());
  std::remove(cifp_path.c_str());
}

TEST_CASE("cifp cache: a corrupt or missing cache is rejected cleanly", "[unit][cifp]") {
  {
    bf::Result<bf::CifpArchive> r = bf::CifpCache::Open("/tmp/bravofinder_no_cifp.bfdb");
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
  }
  {
    const std::string path = TempPath("badmagic_cifp.bfdb");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "XXXXnot a cifp cache";
    f.close();
    bf::Result<bf::CifpArchive> r = bf::CifpCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
  {
    const std::string path = TempPath("trunc_cifp.bfdb");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "BF";  // shorter than the magic
    f.close();
    bf::Result<bf::CifpArchive> r = bf::CifpCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
  // Valid magic, version and header strings, but an absurd airport count. The
  // directory row count must be bounded by the file size before allocating the
  // rows vector, or a forged count would crash instead of a clean Result.
  {
    const std::string path = TempPath("boguscount_cifp.bfdb");
    auto put_u32 = [](std::string& s, uint32_t v) {
      for (int i = 0; i < 4; ++i) {
        s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
      }
    };
    std::string buf;
    buf.append("BFCP", 4);
    put_u32(buf, bf::CifpCache::kFormatVersion);
    put_u32(buf, 0);           // program_semver length (empty)
    put_u32(buf, 0);           // source_loader length (empty)
    put_u32(buf, 2601);        // cycle
    put_u32(buf, 20260112);    // build
    put_u32(buf, 0xFFFFFFFF);  // airport_count: absurd
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f.write(buf.data(), static_cast<std::streamsize>(buf.size()));
    f.close();
    bf::Result<bf::CifpArchive> r = bf::CifpCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
}

TEST_CASE("cifp cache: eager loading yields the same route as on-demand", "[integration][cifp]") {
  const std::string graph_path = BuildBothCaches("eager");
  if (graph_path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string cifp_path = TempPath("eager_nav_cifp.bfdb");

  bf::Result<bf::NavDatabase> on_demand =
      bf::NavDatabase::OpenCached(graph_path, bf::CifpLoad::kOnDemand);
  bf::Result<bf::NavDatabase> eager = bf::NavDatabase::OpenCached(graph_path, bf::CifpLoad::kEager);
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

  std::remove(graph_path.c_str());
  std::remove(cifp_path.c_str());
}

TEST_CASE("cifp cache: concurrent routing on an eager database is race-free",
          "[integration][cifp]") {
  const std::string graph_path = BuildBothCaches("eager_concurrent");
  if (graph_path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string cifp_path = TempPath("eager_concurrent_nav_cifp.bfdb");
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::OpenCached(graph_path, bf::CifpLoad::kEager);
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
  std::remove(graph_path.c_str());
  std::remove(cifp_path.c_str());
}

TEST_CASE("cifp cache: concurrent fetches on one archive are race-free", "[integration][cifp]") {
  if (!HasNavData()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::string cifp_path = TempPath("concurrent_nav_cifp.bfdb");
  bf::Result<uint32_t> n = BuildCifpCache(cifp_path);
  REQUIRE(n);
  bf::Result<bf::CifpArchive> archive = bf::CifpCache::Open(cifp_path);
  REQUIRE(archive);

  // Fetch different airports from many threads at once; Fetch opens its own
  // ifstream per call, so there is no shared mutable state to race on.
  const std::vector<std::string> icaos = {"KJFK", "KLAX", "KSEA", "KBOS",
                                          "KDEN", "KSFO", "KORD", "KDFW"};
  std::atomic<int> found{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([&, t]() {
      for (int rep = 0; rep < 20; ++rep) {
        auto data = archive.value().Fetch(icaos[(t + rep) % icaos.size()]);
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
  std::remove(cifp_path.c_str());
}
