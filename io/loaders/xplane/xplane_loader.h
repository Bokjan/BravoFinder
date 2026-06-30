#pragma once

#include <string>

#include "core/result.h"
#include "io/nav_data.h"

namespace bf {

// Loads navigation data from a directory of X-Plane 12 native ".dat" files
// (earth_fix.dat, earth_nav.dat, earth_awy.dat, earth_aptmeta.dat).
//
// This is the M1 subset: enroute waypoints, navaids, airways, and airport
// reference positions. CIFP procedures are loaded in a later milestone.
class XPlaneLoader {
 public:
  // Parse the four data files under data_dir and return the assembled NavData,
  // or an Error (kDataMissing if a file cannot be opened, kParseError on a
  // malformed file).
  static Result<NavData> Load(const std::string& data_dir);
};

}  // namespace bf
