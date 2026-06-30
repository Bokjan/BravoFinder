#pragma once

#include <string>

namespace bf {

// Direction in which an airway segment may be flown, as encoded in
// earth_awy.dat column 7.
enum class AirwayDirection {
  kBoth,     // 'N' - no restriction, usable in either direction
  kForward,  // 'F' - usable only from -> to
  kBackward  // 'B' - usable only to -> from
};

// Whether a segment belongs to the low (Victor) or high (Jet) airway structure,
// from earth_awy.dat column 8.
enum class AirwayLevel {
  kLow,  // '1'
  kHigh  // '2'
};

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

}  // namespace bf
