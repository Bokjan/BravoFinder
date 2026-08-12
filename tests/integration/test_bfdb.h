// SPDX-License-Identifier: MIT
#pragma once

#include <catch2/catch_test_macros.hpp>
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
// first, so callers need not.
//
// Error contract for callers:
// - kDataMissing — cache file absent → SKIP is appropriate.
// - kFormatMismatch (or any other OpenCached failure) — file exists but is
//   unusable → FAIL the test (do not SKIP; a silent skip hides a poisoned
//   prebuilt cache after a format_version bump).
//
// Note: cache round-trip tests (graph_codec_test / cifp_codec_test /
// unified_cache_test) deliberately build and open their own caches to exercise
// that path and must not use this helper.
inline bf::Result<bf::NavDatabase> OpenReadOnlyDb() {
  const std::string dir = EnsureBfdb();
  const std::string cache = dir + "/nav.bfdb";
  if (std::filesystem::exists(cache)) {
    // Propagate OpenCached errors unchanged (including kFormatMismatch).
    return bf::NavDatabase::OpenCached(cache);
  }
  return bf::Result<bf::NavDatabase>::Err(
      bf::Error(bf::ErrorCode::kDataMissing, "no prebuilt nav.bfdb under " + dir));
}

// Shared opened database for integration suites. Returns nullptr only when the
// prebuilt cache is absent (kDataMissing) so the caller can SKIP. Any other
// open failure — notably kFormatMismatch when nav.bfdb exists — fails the
// current Catch2 test via FAIL (do not treat as SKIP).
inline const bf::NavDatabase* SharedReadOnlyDbOrSkip() {
  static bf::Result<bf::NavDatabase> db = OpenReadOnlyDb();
  if (db) {
    return &db.value();
  }
  if (db.error().code == bf::ErrorCode::kDataMissing) {
    return nullptr;
  }
  FAIL("prebuilt nav.bfdb present but OpenCached failed: " << db.error().message);
  return nullptr;
}

}  // namespace bf::test
