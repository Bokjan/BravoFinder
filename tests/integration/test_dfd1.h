#pragma once

#include <string>

#include "test_db.h"

namespace bf::test {

// Pin the source to the dev checkout's DFD v1.0 directory (a .s3db: navdb /
// navdata / e_dfd_PMDG), overwriting any prior BRAVOFINDER_NAVDATA. The DFD v1
// loader finds its own file inside.
inline std::string EnsureDfd1() {
  SetNavDataDir("navdata/dfd1");
  return NavDataDir();
}

}  // namespace bf::test
