// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <string_view>

namespace bf {

// What a navigation-data source can faithfully express. FindRoutes does not
// currently degrade constraints from these flags (callers / UIs may); they exist
// so consumers do not assume every loader is compliance-equivalent.
//
// Defaults match the full X-Plane 12 / DFD model. Fenix clears airway_direction
// (schema has no per-leg direction) and altitude_bands (segment FL limits are
// not carried the same way). MSA is also absent from Fenix.
struct LoaderCapabilities {
  bool airway_direction = true;  // per-leg F/B/both; false => graph edges are bidirectional
  bool altitude_bands = true;    // per-leg base_fl / top_fl from the source
  bool mora_grid = true;         // global MORA grid present (may still be sparse)
  bool msa_sectors = true;       // per-airport MSA sectors present
};

// Map a loader registry name (or bfdb `source_loader` provenance) to capabilities.
// Unknown names keep the full-capability defaults: a new loader is assumed to
// express every field until proven otherwise (only Fenix clears flags today, and
// it is documented explicitly above).
inline LoaderCapabilities CapabilitiesForLoader(std::string_view loader_name) {
  LoaderCapabilities caps;
  if (loader_name == "fenix") {
    caps.airway_direction = false;
    caps.altitude_bands = false;
    caps.msa_sectors = false;
    // Fenix still loads GridMora when present in the .db3.
  }
  return caps;
}

}  // namespace bf
