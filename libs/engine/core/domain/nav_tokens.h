// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string_view>

namespace bf {

// Stable tokens for filed-flight-plan connectors, CIFP section tags, and the
// ARINC runway-ident prefix. Kept as named string_views so compare/emit sites
// cannot drift by typo (a misspelled "DCT" would silently break round-trips).

inline constexpr std::string_view kDctToken = "DCT";
inline constexpr std::string_view kSidToken = "SID";
inline constexpr std::string_view kStarToken = "STAR";
inline constexpr std::string_view kAppchToken = "APPCH";
inline constexpr std::string_view kRunwayPrefix = "RW";

}  // namespace bf
