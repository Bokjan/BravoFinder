#pragma once

#include <string>
#include <vector>

#include "core/domain/procedure.h"
#include "core/result.h"

namespace bf {

// The procedures and runways parsed from one airport's CIFP file.
struct CifpData {
  std::vector<Procedure> procedures;
  std::vector<Runway> runways;
};

// Parses an X-Plane CIFP file (CIFP/<ICAO>.dat), the ARINC 424-derived terminal
// procedure format. Each line is `TYPE:seq,field,field,...;` where TYPE is one
// of SID/STAR/APPCH/RWY/PRDAT. SID/STAR/APPCH lines are legs; consecutive legs
// sharing (name, transition, route type) form one Procedure. RWY lines give
// runway thresholds. PRDAT (approach metadata) is ignored.
class CifpParser {
 public:
  // Parse the file at `path`. Returns the assembled procedures and runways, or
  // kDataMissing if the file cannot be opened.
  static Result<CifpData> Parse(const std::string& path);

  // Parse CIFP lines from already-loaded text. Exposed for unit testing with
  // small real-format samples (no file I/O).
  static CifpData ParseLines(const std::vector<std::string>& lines);
};

}  // namespace bf
