// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <string_view>

namespace bf {

// Shared suffix appended to cache open/decode error messages so every codec /
// container path steers the user to the same rebuild command.
inline constexpr std::string_view kCacheRebuildHint = "; run bf build to regenerate";

inline std::string WithRebuildHint(std::string_view why) {
  return std::string(why) + std::string(kCacheRebuildHint);
}

}  // namespace bf
