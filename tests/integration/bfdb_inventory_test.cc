#include "io/cache/bfdb_inventory.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <string>

#include "io/cache/bfdb_naming.h"
#include "io/cache/graph_cache.h"
#include "io/cache/graph_snapshot.h"

namespace {

namespace fs = std::filesystem;

// A minimal but valid GraphSnapshot: an empty graph (0 vertices) carrying the given
// AIRAC provenance. Enough for the header-only scan the inventory performs.
bf::GraphSnapshot TinySnapshot(uint32_t cycle, uint32_t build) {
  bf::GraphSnapshot arc;
  arc.cycle = cycle;
  arc.build = build;
  arc.program_semver = "3.2.0";
  arc.source_loader = "test";
  arc.first_airport_vertex = 0;
  arc.offsets = {0};  // CSR offsets for V=0 has size V+1
  // All per-vertex/airport arrays stay empty; MoraGrid defaults to a full grid.
  return arc;
}

// A throwaway directory unique to a test tag, cleaned and recreated so repeated
// runs start empty.
fs::path TempDir(const std::string& tag) {
  const fs::path dir = fs::temp_directory_path() / ("bravofinder_inv_" + tag);
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

// Write a tiny cache under `dir` at the canonical name for cycle/build.
void WriteCache(const fs::path& dir, uint32_t cycle, uint32_t build) {
  const std::string path = (dir / bf::FormatBfdbName(cycle, build)).string();
  REQUIRE(bf::GraphCache::Build(path, TinySnapshot(cycle, build)));
}

}  // namespace

TEST_CASE("inventory: scans a directory of caches indexed by cycle", "[integration][inventory]") {
  const fs::path dir = TempDir("multi");
  WriteCache(dir, 2601, 20260112);
  WriteCache(dir, 2602, 20260209);

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();

  REQUIRE(v.entries().size() == 2);
  // Sorted by cycle ascending.
  CHECK(v.entries()[0].cycle == 2601);
  CHECK(v.entries()[1].cycle == 2602);

  auto latest = v.Latest();
  REQUIRE(latest);
  CHECK(latest->cycle == 2602);
  CHECK(latest->build == 20260209);

  auto found = v.Find(2601);
  REQUIRE(found);
  CHECK(found->build == 20260112);
  CHECK_FALSE(v.Find(9999));
}

TEST_CASE("inventory: keeps the highest build per cycle and records the rest",
          "[integration][inventory]") {
  const fs::path dir = TempDir("dup");
  WriteCache(dir, 2601, 20260112);
  WriteCache(dir, 2601, 20260130);  // a later build of the same cycle wins

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();

  REQUIRE(v.entries().size() == 1);
  CHECK(v.entries()[0].cycle == 2601);
  CHECK(v.entries()[0].build == 20260130);
  // The shadowed lower build is surfaced, not silently dropped.
  REQUIRE(v.discarded().size() == 1);
  CHECK(v.discarded()[0].build == 20260112);
}

TEST_CASE("inventory: header is authoritative over a misleading filename",
          "[integration][inventory]") {
  const fs::path dir = TempDir("rename");
  // A cache whose header says 2605, deliberately stored under a name claiming
  // a different cycle. The scan must trust the header.
  const std::string path = (dir / "nav_2601_20260112.bfdb").string();
  REQUIRE(bf::GraphCache::Build(path, TinySnapshot(2605, 20260501)));

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();

  REQUIRE(v.entries().size() == 1);
  CHECK(v.entries()[0].cycle == 2605);
  CHECK(v.entries()[0].build == 20260501);
  CHECK_FALSE(v.Find(2601));
}

TEST_CASE("inventory: ignores CIFP companions and non-cache files", "[integration][inventory]") {
  const fs::path dir = TempDir("ignore");
  WriteCache(dir, 2601, 20260112);
  // A companion-named file and an unrelated file must both be ignored by name,
  // never opened as graph caches.
  {
    std::ofstream(dir / "nav_2601_20260112_cifp.bfdb") << "not a graph cache";
  }
  {
    std::ofstream(dir / "readme.txt") << "hello";
  }

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();
  REQUIRE(v.entries().size() == 1);
  CHECK(v.entries()[0].cycle == 2601);
  CHECK(v.skipped().empty());  // ignored by name, not opened and skipped
}

TEST_CASE("inventory: records caches with an unreadable header as skipped",
          "[integration][inventory]") {
  const fs::path dir = TempDir("corrupt");
  WriteCache(dir, 2601, 20260112);
  // A file matching the graph-cache name pattern but with garbage contents:
  // opened, header rejected, listed in skipped().
  {
    std::ofstream(dir / "nav_2602_20260209.bfdb") << "GARBAGE not a bfdb";
  }

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();
  REQUIRE(v.entries().size() == 1);
  CHECK(v.entries()[0].cycle == 2601);
  REQUIRE(v.skipped().size() == 1);
}

TEST_CASE("inventory: an empty directory yields no entries", "[integration][inventory]") {
  const fs::path dir = TempDir("empty");
  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  CHECK(inv.value().empty());
  CHECK_FALSE(inv.value().Latest());
}

TEST_CASE("inventory: a missing directory is an error", "[integration][inventory]") {
  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan("/nonexistent/bravofinder/dir/xyzzy");
  CHECK_FALSE(inv);
}
