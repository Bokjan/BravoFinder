// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

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

}  // namespace bf
