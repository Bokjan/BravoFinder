// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string_view>

namespace bf {

// Single-character ARINC 424 / DFD field codes shared across loaders. Named so
// the End-of-Airway pitfall (waypoint_description_code col2 == 'E') and turn /
// course / distance flags cannot be reintroduced as bare literals without the
// surrounding comment context.

// waypoint_description_code column 2: last fix of one same-name airway string.
// Loaders MUST break chaining here; see dfd-loader airway phantom-leg history.
inline constexpr char kWptDescEndOfAirway = 'E';

// Hold / procedure turn direction.
inline constexpr char kTurnLeft = 'L';
inline constexpr char kTurnRight = 'R';

// DFD procedure leg course_flag: 'T' = true course (convert with airport magvar).
inline constexpr char kCourseFlagTrue = 'T';

// DFD procedure leg distance/time flag: 'D' = distance (NM), else time.
inline constexpr char kDistTimeFlagDistance = 'D';

// Match a single character of a fixed-width code field against a code. Used for
// waypoint_description_code, whose End-of-Airway marker sits in column 2 (index
// 1), not as the whole field -- that distinction is why EqualsCode (whole-field
// match) is the separate, stricter form. Bounds-checked so a short field is
// never a hit.
inline bool HasCodeAt(std::string_view s, size_t pos, char code) {
  return pos < s.size() && s[pos] == code;
}

// Match a column string against a single-char code. The DFD columns store these
// as exactly-one-character strings; a multi-char value is never a match, so a
// bare string literal comparison ("D") would be equivalent but names nothing.
// Expresses the match as HasCodeAt(s, 0, code) with the extra constraint that
// the field is exactly one character (HasCodeAt alone would also accept a
// multi-char field whose leading column matches).
inline bool EqualsCode(std::string_view s, char code) {
  return s.size() == 1 && HasCodeAt(s, 0, code);
}

}  // namespace bf
