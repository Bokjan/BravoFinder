#pragma once

#include <string>
#include <utility>
#include <vector>

#include "core/result.h"
#include "io/loaders/xplane/cifp/cifp_parser.h"
#include "io/nav_data.h"

namespace bf {

// One airport's parsed CIFP data, keyed by ICAO. LoadProcedures returns these
// in directory-iteration order; the ICAO comes from the file stem.
using AirportProcedureData = std::pair<std::string, CifpData>;

// Loads navigation data from a directory of X-Plane 12 native ".dat" files
// (earth_fix.dat, earth_nav.dat, earth_awy.dat, earth_aptmeta.dat).
//
// This is the M1 subset: enroute waypoints, navaids, airways, and airport
// reference positions. CIFP procedures are loaded in a later milestone.
class XPlaneLoader {
 public:
  // Parse the data files under data_dir and return the assembled NavData, or an
  // Error (kDataMissing if a file cannot be opened, kParseError on a malformed
  // file). This is the lightweight path: it does NOT parse CIFP procedures (see
  // LoadProcedures), so route/query stay cheap.
  static Result<NavData> Load(const std::string& data_dir);

  // Parse every CIFP/<ICAO>.dat file under data_dir into per-airport procedure
  // data. This is the heavy path (~100 MB for a full cycle) used only by
  // `bf build` to feed CifpCache::Build; it is kept out of Load so the common
  // route/query paths never pay for it. Files that fail to parse are skipped.
  // Returns kDataMissing if there is no CIFP directory.
  static Result<std::vector<AirportProcedureData>> LoadProcedures(const std::string& data_dir);
};

}  // namespace bf
