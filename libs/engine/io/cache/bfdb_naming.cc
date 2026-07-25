// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/bfdb_naming.h"

#include <filesystem>

namespace bf {

std::string FormatBfdbName(uint32_t cycle) {
  if (cycle == 0) {
    return "nav.bfdb";
  }
  return "nav_" + std::to_string(cycle) + ".bfdb";
}

std::optional<uint32_t> ParseBfdbName(std::string_view path) {
  // Inspect only the filename component, so a full path parses the same as a
  // bare name.
  const std::string name = std::filesystem::path(path).filename().string();

  // The zero-cycle sentinel (data with no parsed AIRAC provenance) is written by
  // FormatBfdbName as the legacy "nav.bfdb". Accept it so Format/Parse stay
  // symmetric and inventory discovery does not silently drop such a cache.
  if (name == "nav.bfdb") {
    return 0;
  }

  // Require the exact shape "nav_<digits>.bfdb".
  constexpr std::string_view kPrefix = "nav_";
  constexpr std::string_view kSuffix = ".bfdb";
  if (name.size() <= kPrefix.size() + kSuffix.size() ||
      name.compare(0, kPrefix.size(), kPrefix) != 0 ||
      name.compare(name.size() - kSuffix.size(), kSuffix.size(), kSuffix) != 0) {
    return std::nullopt;
  }
  const std::string_view cycle_str(name.data() + kPrefix.size(),
                                   name.size() - kPrefix.size() - kSuffix.size());
  if (cycle_str.empty()) {
    return std::nullopt;
  }

  // The cycle segment must be all digits and fit in a uint32_t.
  uint64_t value = 0;
  for (const char c : cycle_str) {
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    value = value * 10 + static_cast<uint64_t>(c - '0');
    if (value > 0xFFFFFFFFULL) {
      return std::nullopt;
    }
  }
  return static_cast<uint32_t>(value);
}

}  // namespace bf
