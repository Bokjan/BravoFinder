// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/domain/procedure.h"
#include "core/result.h"

namespace bf {

// Parses an X-Plane CIFP file (CIFP/<ICAO>.dat), the ARINC 424-derived terminal
// procedure format. Each line is `TYPE:seq,field,field,...;` where TYPE is one
// of SID/STAR/APPCH/RWY/PRDAT. SID/STAR/APPCH lines are legs; consecutive legs
// sharing (name, transition, route type) form one Procedure. RWY lines give
// runway thresholds. PRDAT (approach metadata) is ignored.
//
// `CifpData` (the procedures + runways assembled here) is defined in
// core/domain/procedure.h so the DFD SQLite loaders can produce it without
// depending on the X-Plane CIFP parser.
//
// A CIFP leg names its fix by (ident, region) plus the ARINC 424 section and
// subsection of the record it points at (e.g. 'P','C' = this airport's terminal
// waypoint, 'E','A' = enroute waypoint, 'D',' ' = VHF navaid, 'D','B' = NDB) but
// carries no position. A FixLocator maps that reference to the fix's published
// position so the leg can record it (ProcedureLeg::fix_lat/fix_lon); without one
// the legs carry no position.
using FixLocator = std::function<std::optional<Coordinate>(
    std::string_view ident, std::string_view region, char section, char subsection)>;

class CifpParser {
 public:
  // Parse the file at `path`. Returns the assembled procedures and runways, or
  // kDataMissing if the file cannot be opened.
  static Result<CifpData> Parse(const std::string& path, const FixLocator& locate = {});

  // Parse CIFP lines from already-loaded text. Exposed for unit testing with
  // small real-format samples (no file I/O).
  static CifpData ParseLines(const std::vector<std::string>& lines, const FixLocator& locate = {});
};

}  // namespace bf
