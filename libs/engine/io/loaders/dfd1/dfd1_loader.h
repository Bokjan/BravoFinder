// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "io/loaders/loader.h"

namespace bf {

// DFD v1.0 SQLite loader. Parses the "Data File Definition v1.0" SQLite format
// (RealTraffic / SimToolkitPro / PMDG MSFS 737/777 `navdb.s3db`,
// `navdata.s3db`, or `e_dfd_PMDG.s3db` -- byte-identical copies of the same
// Jeppesen dataset) into the same NavData / CifpData domain types as
// XPlane12Loader. Read-only; thread-safe (the underlying SQLite connection is
// held per-thread, never shared -- see sqlite_util).
//
// `source_dir` points at the directory containing the .s3db file. The loader
// searches for navdb.s3db -> navdata.s3db -> e_dfd_PMDG.s3db -> first *.s3db.
class Dfd1Loader final : public Loader {
 public:
  Result<NavData> LoadNavData(const std::string& source_dir) const override;
  Result<std::vector<AirportProcedureData>> LoadProcedures(
      const std::string& source_dir) const override;
  std::optional<CifpData> LoadProcedure(const std::string& source_dir,
                                        const std::string& icao) const override;
  std::string name() const override { return "dfd1"; }
};

}  // namespace bf
