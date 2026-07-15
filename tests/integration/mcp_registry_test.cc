#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include "io/cache/bfdb_inventory.h"
#include "io/cache/bfdb_naming.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/unified_cache.h"
#include "registry.h"

namespace {

namespace fs = std::filesystem;

// A minimal valid graph-only unified cache carrying an AIRAC cycle. Enough for
// OpenCached to build a (trivial) NavDatabase; the registry only cares that it
// opens.
void WriteCache(const fs::path& dir, uint32_t cycle) {
  bf::GraphSnapshot g;
  g.first_airport_vertex = 0;
  g.offsets = {0};
  bf::UnifiedCache::BuildInput in;
  in.graph = &g;
  in.header.cycle = cycle;
  in.header.program_semver = "3.3.0";
  in.header.source_loader = "test";
  const std::string path = (dir / bf::FormatBfdbName(cycle)).string();
  REQUIRE(bf::UnifiedCache::Build(path, in));
}

fs::path TempDir(const std::string& tag) {
  const fs::path dir = fs::temp_directory_path() / ("bravofinder_reg_" + tag);
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

bf::service::NavDatabaseRegistry MakeRegistry(const fs::path& dir) {
  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  return bf::service::NavDatabaseRegistry(std::move(inv.value()));
}

}  // namespace

TEST_CASE("mcp registry: Get with no cycle serves the latest", "[integration][mcp]") {
  const fs::path dir = TempDir("latest");
  WriteCache(dir, 2601);
  WriteCache(dir, 2602);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);

  bf::Result<const bf::NavDatabase*> db = reg.Get(std::nullopt);
  REQUIRE(db);
  CHECK(db.value()->cycle() == 2602);
}

TEST_CASE("mcp registry: Get by cycle serves that cycle", "[integration][mcp]") {
  const fs::path dir = TempDir("bycycle");
  WriteCache(dir, 2601);
  WriteCache(dir, 2602);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);

  bf::Result<const bf::NavDatabase*> db = reg.Get(2601);
  REQUIRE(db);
  CHECK(db.value()->cycle() == 2601);
}

TEST_CASE("mcp registry: repeated Get returns the same cached instance", "[integration][mcp]") {
  const fs::path dir = TempDir("cache");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);

  bf::Result<const bf::NavDatabase*> a = reg.Get(2601);
  bf::Result<const bf::NavDatabase*> b = reg.Get(2601);
  REQUIRE(a);
  REQUIRE(b);
  CHECK(a.value() == b.value());  // same pointer: opened once, cached
}

TEST_CASE("mcp registry: unknown cycle is an error", "[integration][mcp]") {
  const fs::path dir = TempDir("unknown");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);

  CHECK_FALSE(reg.Get(9999));
}

TEST_CASE("mcp registry: concurrent Get is safe and consistent", "[integration][mcp]") {
  // Exercises the lazy-open path under contention: many threads race to open
  // the same and different cycles. Run under the tsan preset to check for data
  // races on the cache map. Threads must NOT call Catch2 assertion macros --
  // those are not thread-safe -- so each stores its result and we assert after
  // the join.
  const fs::path dir = TempDir("concurrent");
  WriteCache(dir, 2601);
  WriteCache(dir, 2602);
  WriteCache(dir, 2603);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);

  constexpr int kThreads = 8;
  std::vector<std::thread> threads;
  std::vector<const bf::NavDatabase*> results(kThreads, nullptr);
  std::vector<uint32_t> requested(kThreads, 0);
  for (int t = 0; t < kThreads; ++t) {
    requested[t] = 2601 + static_cast<uint32_t>(t % 3);
    threads.emplace_back([&, t]() {
      bf::Result<const bf::NavDatabase*> db = reg.Get(requested[t]);
      results[t] = db ? db.value() : nullptr;
    });
  }
  for (std::thread& th : threads) {
    th.join();
  }
  // Every thread must have opened its cycle, and all requests for a given cycle
  // must have received the one cached instance.
  const bf::NavDatabase* canonical_2601 = reg.Get(2601).value();
  for (int t = 0; t < kThreads; ++t) {
    REQUIRE(results[t] != nullptr);
    CHECK(results[t]->cycle() == requested[t]);
    if (requested[t] == 2601) {
      CHECK(results[t] == canonical_2601);
    }
  }
}
