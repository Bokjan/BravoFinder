// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "io/loaders/loader.h"

namespace bf {

// DFD v2 SQLite loader. Parses the "NG_FWDFD" v2 SQLite format (Inibuilds A350
// `db.s3db`) into the same NavData / CifpData domain types as XPlane12Loader.
// Read-only and thread-safe, like Dfd1Loader.
//
// v2 differs from v1.0 in table-name prefixes (tbl_pa_*, tbl_d_*, ...), several
// column renames (vor_* -> navaid_*, magnetic_course -> course + course_flag,
// recommanded_* -> recommended_*), a swapped distance/value+flag column pair,
// and a quadrant_code subdivision on the MORA grid. These are all handled inside
// the loader; downstream code is unchanged.
//
// `source_dir` points at the directory containing db.s3db (Inibuilds extracts it
// under NavigationData/db.s3db).
class Dfd2Loader final : public Loader {
 public:
  Result<NavData> LoadNavData(const std::string& source_dir) const override;
  Result<std::vector<AirportProcedureData>> LoadProcedures(
      const std::string& source_dir) const override;
  std::optional<CifpData> LoadProcedure(const std::string& source_dir,
                                        const std::string& icao) const override;
  std::string name() const override { return "dfd2"; }
};

// Convert a true course to magnetic using the airport's magnetic variation
// (west negative, so magnetic > true when variation is west). The v2 loader
// applies this to 'T' (true-course) procedure legs; exposed here so the sign
// convention is locked by a unit test.
double ToMagnetic(double true_course, double magvar);

}  // namespace bf
