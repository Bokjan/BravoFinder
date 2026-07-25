// SPDX-License-Identifier: MIT
#pragma once

#include <filesystem>
#include <string>

#include "io/nav_database.h"
#include "test_db.h"

namespace bf::test {

// Pin the source to the dev checkout's prebuilt-cache directory, overwriting
// any prior BRAVOFINDER_NAVDATA. Used by pure-domain tests (query, mcp_registry)
// that only read a ready NavDatabase and do not care which loader produced it.
inline std::string EnsureBfdb() {
  SetNavDataDir("navdata/bfdb");
  return NavDataDir();
}

// Open a database for read-only integration tests from the prebuilt
// <bfdb>/nav.bfdb cache (loads in ~1.5s). Pins the source to the bfdb directory
// first, so callers need not. Returns an errored Result when the cache is
// absent so callers can SKIP.
//
// Note: cache round-trip tests (graph_codec_test / cifp_codec_test /
// unified_cache_test) deliberately build and open their own caches to exercise
// that path and must not use this helper.
inline bf::Result<bf::NavDatabase> OpenReadOnlyDb() {
  const std::string dir = EnsureBfdb();
  const std::string cache = dir + "/nav.bfdb";
  if (std::filesystem::exists(cache)) {
    return bf::NavDatabase::OpenCached(cache);
  }
  return bf::Result<bf::NavDatabase>::Err(
      bf::Error(bf::ErrorCode::kDataMissing, "no prebuilt nav.bfdb under " + dir));
}

}  // namespace bf::test
