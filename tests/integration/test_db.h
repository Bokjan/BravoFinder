// SPDX-License-Identifier: MIT
#pragma once

#include <filesystem>
#include <string>

#include "core/env.h"

namespace bf::test {

// The navigation-data directory currently selected (the value of
// BRAVOFINDER_NAVDATA, or "navdata" by default). Read fresh each call so a test
// can switch sources at runtime via SetNavDataDir / SetEnv.
inline std::string NavDataDir() {
  if (const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA")) {
    return env;
  }
  return "navdata";
}

// Set the navigation-data directory for the current process (and its children).
inline void SetNavDataDir(const std::string& dir) { bf::SetEnv("BRAVOFINDER_NAVDATA", dir); }

// Whether `dir` holds at least one *.s3db file (a DFD SQLite database). Real
// Jeppesen data is never committed, so the DFD test helpers use this to return
// an empty path and let callers SKIP when the data is absent.
inline bool HasS3db(const std::string& dir) {
  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec)) {
    return false;
  }
  for (const auto& de : std::filesystem::directory_iterator(dir, ec)) {
    if (de.is_regular_file() && de.path().extension() == ".s3db") {
      return true;
    }
  }
  return false;
}

// Whether `dir` holds at least one *.db3 file (a Fenix SQLite database).
// Mirrors HasS3db for the Fenix loader's per-loader subdirectory convention.
inline bool HasDb3(const std::string& dir) {
  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec)) {
    return false;
  }
  for (const auto& de : std::filesystem::directory_iterator(dir, ec)) {
    if (de.is_regular_file() && de.path().extension() == ".db3") {
      return true;
    }
  }
  return false;
}

}  // namespace bf::test
