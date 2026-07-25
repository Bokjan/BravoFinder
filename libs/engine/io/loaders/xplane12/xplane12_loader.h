// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "core/result.h"
#include "io/loaders/loader.h"
#include "io/loaders/xplane12/cifp_parser.h"
#include "io/nav_data.h"

namespace bf {

// Loads navigation data from a directory of X-Plane 12 native ".dat" files
// (earth_fix.dat, earth_nav.dat, earth_awy.dat, earth_aptmeta.dat, the optional
// earth_mora/msa/hold files, and CIFP/<ICAO>.dat procedures).
//
// This loader targets the X-Plane *12* native format specifically (header
// "1200 Version", 11-column earth_nav rows, earth_hold.dat): the X-Plane 11
// layout differs and is not supported. The name "xplane12" reflects that.
class XPlane12Loader final : public Loader {
 public:
  // Parse earth_*.dat under source_dir into NavData, or an Error (kDataMissing
  // if a required file cannot be opened, kParseError on a malformed file). Does
  // NOT parse CIFP procedures (see LoadProcedures / LoadProcedure), so the
  // common route/query paths stay cheap.
  Result<NavData> LoadNavData(const std::string& source_dir) const override;

  // Parse every CIFP/<ICAO>.dat file under source_dir into per-airport procedure
  // data (~100 MB for a full cycle). Files that fail to parse are skipped.
  // Returns kDataMissing if there is no CIFP directory.
  Result<std::vector<AirportProcedureData>> LoadProcedures(
      const std::string& source_dir) const override;

  // Parse a single CIFP/<ICAO>.dat on demand, or nullopt if that file is absent
  // or unreadable.
  std::optional<CifpData> LoadProcedure(const std::string& source_dir,
                                        const std::string& icao) const override;

  std::string name() const override { return "xplane12"; }
};

}  // namespace bf
