// SPDX-License-Identifier: MIT
#pragma once

#include <string>

#include "test_db.h"

namespace bf::test {

// Pin the source to the dev checkout's DFD v2 directory (db.s3db), overwriting
// any prior BRAVOFINDER_NAVDATA. The DFD v2 loader finds its own file inside.
// Returns the directory when it holds a .s3db, or an empty string when the data
// is absent (real Jeppesen data is not committed) so callers can SKIP.
inline std::string EnsureDfd2() {
  SetNavDataDir("navdata/dfd2");
  const std::string dir = NavDataDir();
  return HasS3db(dir) ? dir : std::string{};
}

}  // namespace bf::test
