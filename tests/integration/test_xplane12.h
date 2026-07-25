// SPDX-License-Identifier: MIT
#pragma once

#include <fstream>
#include <string>

#include "io/nav_database.h"
#include "test_db.h"

namespace bf::test {

// Pin the source to the dev checkout's X-Plane source directory (earth_*.dat +
// CIFP/), overwriting any prior BRAVOFINDER_NAVDATA. Used by tests that build a
// cache from raw X-Plane data (graph_codec, cifp_codec) or check for CIFP files.
inline std::string EnsureXPlane12() {
  SetNavDataDir("navdata/xplane12");
  return NavDataDir();
}

// Whether <dir>/CIFP/<icao>.dat exists.
inline bool HasCifp(const std::string& dir, const std::string& icao) {
  std::ifstream f(dir + "/CIFP/" + icao + ".dat");
  return f.is_open();
}

// Whether <dir>/earth_fix.dat exists (the X-Plane source is present).
inline bool HasNavData(const std::string& dir) {
  std::ifstream f(dir + "/earth_fix.dat");
  return f.is_open();
}

}  // namespace bf::test
