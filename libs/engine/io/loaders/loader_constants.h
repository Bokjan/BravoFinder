// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Shared numeric conventions used by more than one navigation-data loader
// (DFD v1/v2, Fenix). Kept out of sqlite_util.h so that header stays pure
// SQLite plumbing.

namespace bf {

// DFD / Fenix encode "no ceiling" / unset max altitude as this value in feet.
// HoldFix and similar domain types use 0 for the same meaning (matching X-Plane).
inline constexpr int kUnknownAltitudeFt = 99999;

// Flight level = hundreds of feet MSL.
inline constexpr int kFeetPerFlightLevel = 100;

}  // namespace bf
