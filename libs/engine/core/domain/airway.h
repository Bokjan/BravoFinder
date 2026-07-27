// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace bf {

// Direction in which an airway segment may be flown, as encoded in
// earth_awy.dat column 7 (or DFD `direction_restriction`).
enum class AirwayDirection {
  kBoth,     // 'N' - no restriction, usable in either direction
  kForward,  // 'F' - usable only from -> to
  kBackward  // 'B' - usable only to -> from
};

// Whether a segment belongs to the low (Victor) or high (Jet) airway structure,
// or both. X-Plane `earth_awy.dat` uses integer 1/2; DFD `flightlevel` uses
// 'L'/'H'/'B' ('B' occurs for roughly 1/3 of segments and must be honored -- see the DFD loader
// plan).
//
// The underlying type is uint8_t so the value fits in a single byte on a
// GraphEdge. The integer values are part of the cache format: the edge 'level'
// field is serialized as this raw value, so reordering the enumerators would
// silently corrupt existing caches. The static_assert below pins them.
enum class AirwayLevel : uint8_t {
  kLow,   // '1' / 'L'
  kHigh,  // '2' / 'H'
  kBoth,  // 'B' - usable at both low and high (DFD only; X-Plane has no both)
};
static_assert(static_cast<uint8_t>(AirwayLevel::kLow) == 0 &&
                  static_cast<uint8_t>(AirwayLevel::kHigh) == 1 &&
                  static_cast<uint8_t>(AirwayLevel::kBoth) == 2,
              "AirwayLevel values are part of the cache format; do not reorder");

// A single airway segment connecting two adjacent waypoints. One named airway
// (e.g. "J80") is made of many such segments laid end to end. Altitudes are in
// flight levels (hundreds of feet) as stored in the source data.
struct AirwaySegment {
  std::string name;  // airway name, e.g. "J80" (column 11)
  AirwayDirection direction = AirwayDirection::kBoth;
  AirwayLevel level = AirwayLevel::kLow;
  int base_fl = 0;  // lowest usable flight level (column 9)
  int top_fl = 0;   // highest usable flight level (column 10)
};

// Parse an airway direction token ('F'/'B'/blank) from earth_awy.dat or DFD.
// Blank or any unrecognized value means no restriction (kBoth). Shared so X-Plane
// and the DFD loaders agree on the mapping.
inline AirwayDirection ParseDirection(std::string_view token) {
  if (token == "F") {
    return AirwayDirection::kForward;
  }
  if (token == "B") {
    return AirwayDirection::kBackward;
  }
  return AirwayDirection::kBoth;  // 'N' or blank
}

// Parse an airway level token. Accepts X-Plane's '1'/'2' (as text) and DFD's
// 'L'/'H'/'B'. Used by the DFD loaders; X-Plane reads its integer hilo column
// directly. 'B' (both) maps to kBoth so it is never penalized by level filtering.
inline AirwayLevel ParseAirwayLevel(std::string_view token) {
  if (token == "2" || token == "H") {
    return AirwayLevel::kHigh;
  }
  if (token == "B") {
    return AirwayLevel::kBoth;
  }
  return AirwayLevel::kLow;  // '1' or 'L'
}

}  // namespace bf
