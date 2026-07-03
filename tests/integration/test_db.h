#pragma once

#include <filesystem>
#include <string>

#include "core/env.h"
#include "io/nav_database.h"

namespace bf::test {

// Resolve the navigation data directory: BRAVOFINDER_NAVDATA if set, else the
// repository's navdata/ folder. Real Navigraph/Jeppesen data is not committed,
// so tests SKIP (rather than fail) when the data is absent.
inline std::string NavDataDir() {
  if (const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA")) {
    return env;
  }
  return "navdata";
}

// Open a database for read-only integration tests, preferring a prebuilt
// `<dir>/nav.bfdb` cache (loads in ~1.5s) over parsing the raw .dat files
// (~8.5s). The cache path auto-discovers sibling `nav_cifp.bfdb` / `nav_detail.bfdb`
// companions for procedures and navaid detail. Falls back to Open() when no
// graph cache is present, so a fresh checkout without a built cache still runs
// (just slower). Behavior is identical either way; only load time differs.
// Returns an errored Result when the data is absent so callers can SKIP.
//
// Note: this is for tests that only READ the database. The cache round-trip
// tests (graph_cache_test / cifp_cache_test) deliberately build and open their
// own caches to exercise that path and must not use this helper.
inline bf::Result<bf::NavDatabase> OpenReadOnlyDb(const std::string& dir) {
  const std::string cache = dir + "/nav.bfdb";
  if (std::filesystem::exists(cache)) {
    return bf::NavDatabase::OpenCached(cache);
  }
  return bf::NavDatabase::Open(dir);
}

}  // namespace bf::test
