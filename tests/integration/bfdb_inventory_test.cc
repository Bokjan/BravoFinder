#include "io/cache/bfdb_inventory.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <string>

#include "io/cache/bfdb_naming.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/unified_cache.h"

namespace {

namespace fs = std::filesystem;

// A minimal but valid graph snapshot: an empty graph (0 vertices). Enough for
// the header-only scan the inventory performs.
bf::GraphSnapshot TinyGraph() {
  bf::GraphSnapshot g;
  g.first_airport_vertex = 0;
  g.offsets = {0};  // CSR offsets for V=0 has size V+1
  // All per-vertex/airport arrays stay empty; MoraGrid defaults to a full grid.
  return g;
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

// Write a tiny graph-only unified cache carrying `cycle` in its header, at
// `path`.
void WriteCacheAt(const std::string& path, uint32_t cycle) {
  const bf::GraphSnapshot g = TinyGraph();
  bf::UnifiedCache::BuildInput in;
  in.graph = &g;
  in.header.cycle = cycle;
  in.header.program_semver = "3.3.0";
  in.header.source_loader = "test";
  REQUIRE(bf::UnifiedCache::Build(path, in));
}

// Write a tiny cache under `dir` at the canonical name for `cycle`.
void WriteCache(const fs::path& dir, uint32_t cycle) {
  WriteCacheAt((dir / bf::FormatBfdbName(cycle)).string(), cycle);
}

}  // namespace

TEST_CASE("inventory: scans a directory of caches indexed by cycle", "[integration][inventory]") {
  const fs::path dir = TempDir("multi");
  WriteCache(dir, 2601);
  WriteCache(dir, 2602);

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

  auto found = v.Find(2601);
  REQUIRE(found);
  CHECK(found->cycle == 2601);
  CHECK_FALSE(v.Find(9999));
}

TEST_CASE("inventory: same-cycle duplicates keep one and record the rest",
          "[integration][inventory]") {
  const fs::path dir = TempDir("dup");
  // Two files with the same header cycle but different names. Only one wins per
  // cycle; the other is surfaced in discarded(), not silently dropped. (There is
  // no build stamp to rank them by anymore.)
  WriteCacheAt((dir / "nav_2601.bfdb").string(), 2601);
  WriteCacheAt((dir / "nav_9998.bfdb").string(), 2601);  // header says 2601 too

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();

  REQUIRE(v.entries().size() == 1);
  CHECK(v.entries()[0].cycle == 2601);
  REQUIRE(v.discarded().size() == 1);
  CHECK(v.discarded()[0].cycle == 2601);
}

TEST_CASE("inventory: header is authoritative over a misleading filename",
          "[integration][inventory]") {
  const fs::path dir = TempDir("rename");
  // A cache whose header says 2605, deliberately stored under a name claiming a
  // different cycle. The scan must trust the header.
  WriteCacheAt((dir / "nav_2601.bfdb").string(), 2605);

  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  const bf::BfdbInventory& v = inv.value();

  REQUIRE(v.entries().size() == 1);
  CHECK(v.entries()[0].cycle == 2605);
  CHECK_FALSE(v.Find(2601));
}

TEST_CASE("inventory: ignores non-cache files", "[integration][inventory]") {
  const fs::path dir = TempDir("ignore");
  WriteCache(dir, 2601);
  // An unrelated file must be ignored by name, never opened as a cache.
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
  WriteCache(dir, 2601);
  // A file matching the cache name pattern but with garbage contents: opened,
  // header rejected, listed in skipped().
  {
    std::ofstream(dir / "nav_2602.bfdb") << "GARBAGE not a bfdb";
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
