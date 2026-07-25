// SPDX-License-Identifier: MIT
#pragma once

#include <string>

#include "test_db.h"

namespace bf::test {

// Pin the source to the dev checkout's DFD v1.0 directory (a .s3db: navdb /
// navdata / e_dfd_PMDG), overwriting any prior BRAVOFINDER_NAVDATA. The DFD v1
// loader finds its own file inside. Returns the directory when it holds a
// .s3db, or an empty string when the data is absent (real Jeppesen data is not
// committed) so callers can SKIP -- mirroring EnsureXPlane12 + HasNavData.
inline std::string EnsureDfd1() {
  SetNavDataDir("navdata/dfd1");
  const std::string dir = NavDataDir();
  return HasS3db(dir) ? dir : std::string{};
}

}  // namespace bf::test
