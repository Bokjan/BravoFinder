#pragma once

#include <string>

#include "test_db.h"

namespace bf::test {

// Pin the source to the dev checkout's DFD v2 directory (db.s3db), overwriting
// any prior BRAVOFINDER_NAVDATA. The DFD v2 loader finds its own file inside.
inline std::string EnsureDfd2() {
  SetNavDataDir("navdata/dfd2");
  return NavDataDir();
}

}  // namespace bf::test
