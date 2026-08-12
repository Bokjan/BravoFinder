// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "io/loaders/loader.h"

namespace bf {

// Fenix A320 SQLite loader. Parses the Fenix flight-management database
// (typically `fenix_navdata.db3`) into the same NavData / CifpData domain
// types as the other loaders.
//
// `source_dir` points at the directory containing the .db3 file. The loader
// searches for fenix_navdata.db3 first, then navdata.db3 and fenix.db3,
// falling back to the first *.db3 found.
class FenixLoader final : public Loader {
 public:
  Result<NavData> LoadNavData(const std::string& source_dir) const override;
  Result<std::vector<AirportProcedureData>> LoadProcedures(
      const std::string& source_dir) const override;
  std::optional<CifpData> LoadProcedure(const std::string& source_dir,
                                        const std::string& icao) const override;
  std::string name() const override { return "fenix"; }

  // Fenix schema has no per-leg airway direction or altitude bands, and does
  // not publish MSA. MORA is still loaded when present in the .db3.
  LoaderCapabilities capabilities() const override {
    LoaderCapabilities caps;
    caps.airway_direction = false;
    caps.altitude_bands = false;
    caps.msa_sectors = false;
    return caps;
  }
};

}  // namespace bf
