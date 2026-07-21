#include "cli_common.h"

#include <charconv>

namespace bf::cli {

Result<NavDatabase> OpenForRead(const std::string& db_path, const std::string& data_dir,
                                const std::string& cifp_load) {
  if (db_path.empty()) {
    return NavDatabase::Open(data_dir);
  }
  return NavDatabase::OpenCached(db_path,
                                 cifp_load == "eager" ? CifpLoad::kEager : CifpLoad::kOnDemand);
}

std::optional<FlRange> ParseAltSpec(const std::string& spec) {
  const size_t dash = spec.find('-');
  auto to_int = [](const std::string& s, int& out) -> bool {
    if (s.empty()) {
      return false;
    }
    // std::from_chars is the exception-free counterpart of stoi: it fails via
    // an error code (no try/catch), and ptr == end verifies the whole field
    // was numeric. A leading '+' or whitespace is rejected, which is fine for
    // a flight-level spec.
    const char* begin = s.data();
    const char* end = begin + s.size();
    auto [ptr, ec] = std::from_chars(begin, end, out);
    return ec == std::errc{} && ptr == end && out >= 0;
  };
  if (dash == std::string::npos) {
    int fl = 0;
    if (!to_int(spec, fl)) {
      return std::nullopt;
    }
    return FlRange{fl, fl};
  }
  int lo = 0;
  int hi = 0;
  if (!to_int(spec.substr(0, dash), lo) || !to_int(spec.substr(dash + 1), hi)) {
    return std::nullopt;
  }
  if (lo > hi) {
    return std::nullopt;
  }
  return FlRange{lo, hi};
}

}  // namespace bf::cli
