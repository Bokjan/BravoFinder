#include "io/cache/bfdb_naming.h"

#include <filesystem>

namespace bf {

std::string FormatBfdbName(uint32_t cycle, uint32_t build) {
  if (cycle == 0 && build == 0) {
    return "nav.bfdb";
  }
  return "nav_" + std::to_string(cycle) + "_" + std::to_string(build) + ".bfdb";
}

std::optional<std::pair<uint32_t, uint32_t>> ParseBfdbName(std::string_view path) {
  // Inspect only the filename component, so a full path parses the same as a
  // bare name.
  const std::string name = std::filesystem::path(path).filename().string();

  // Require the exact shape "nav_<digits>_<digits>.bfdb". Anything else -- most
  // notably the "<stem>_cifp.bfdb" companion, whose second segment is "cifp" --
  // is not a graph cache name and returns nullopt.
  constexpr std::string_view kPrefix = "nav_";
  constexpr std::string_view kSuffix = ".bfdb";
  if (name.size() <= kPrefix.size() + kSuffix.size() || name.compare(0, kPrefix.size(), kPrefix) != 0 ||
      name.compare(name.size() - kSuffix.size(), kSuffix.size(), kSuffix) != 0) {
    return std::nullopt;
  }
  const std::string_view body(name.data() + kPrefix.size(),
                              name.size() - kPrefix.size() - kSuffix.size());

  // Split "<cycle>_<build>" on the single underscore between the two numbers.
  const size_t sep = body.find('_');
  if (sep == std::string_view::npos) {
    return std::nullopt;
  }
  const std::string_view cycle_str = body.substr(0, sep);
  const std::string_view build_str = body.substr(sep + 1);
  if (cycle_str.empty() || build_str.empty()) {
    return std::nullopt;
  }

  // Both segments must be all digits and fit in a uint32_t; reject otherwise so
  // "nav_foo_cifp.bfdb" and overflowing numbers do not parse.
  auto to_u32 = [](std::string_view s) -> std::optional<uint32_t> {
    uint64_t value = 0;
    for (const char c : s) {
      if (c < '0' || c > '9') {
        return std::nullopt;
      }
      value = value * 10 + static_cast<uint64_t>(c - '0');
      if (value > 0xFFFFFFFFULL) {
        return std::nullopt;
      }
    }
    return static_cast<uint32_t>(value);
  };
  const std::optional<uint32_t> cycle = to_u32(cycle_str);
  const std::optional<uint32_t> build = to_u32(build_str);
  if (!cycle || !build) {
    return std::nullopt;
  }
  return std::make_pair(*cycle, *build);
}

}  // namespace bf
