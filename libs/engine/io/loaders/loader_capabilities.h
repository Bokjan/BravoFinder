// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

namespace bf {

// What a navigation-data source can faithfully express. Declared by each
// Loader via `capabilities()`, written into the .bfdb container header at
// `bf build`, and restored on OpenCached. FindRoutes does not currently degrade
// constraints from these flags (callers / UIs may); they exist so consumers do
// not assume every loader is compliance-equivalent.
//
// Defaults match the full X-Plane 12 / DFD model. Fenix clears airway_direction
// (schema has no per-leg direction) and altitude_bands (segment FL limits are
// not carried the same way). MSA is also absent from Fenix.
//
// This type is a public SDK leaf (NavDatabase::capabilities()) and must not pull
// in the full Loader interface — keep it in this header alone.
struct LoaderCapabilities {
  bool airway_direction = true;  // per-leg F/B/both; false => graph edges are bidirectional
  bool altitude_bands = true;    // per-leg base_fl / top_fl from the source
  bool mora_grid = true;         // global MORA grid present (may still be sparse)
  bool msa_sectors = true;       // per-airport MSA sectors present
};

}  // namespace bf
