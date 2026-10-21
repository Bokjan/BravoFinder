// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/fs_util.h"

#include <algorithm>
#include <filesystem>
#include <format>
#include <system_error>
#include <utility>

namespace bf {

Result<std::vector<std::string>> ListFiles(const std::string& dir, std::string_view ext) {
  namespace fs = std::filesystem;
  auto unreadable = [&](const std::error_code& ec) {
    return Result<std::vector<std::string>>::Err(Error(
        ErrorCode::kDataMissing, std::format("cannot read directory: {} ({})", dir, ec.message())));
  };
  std::error_code ec;
  fs::directory_iterator it(dir, ec);
  if (ec) {
    return unreadable(ec);
  }
  const fs::path wanted_ext(ext);
  std::vector<std::string> out;
  for (; it != fs::directory_iterator(); it.increment(ec)) {
    if (ec) {
      return unreadable(ec);
    }
    const fs::path& p = it->path();
    std::error_code type_ec;
    if (p.extension() != wanted_ext || !it->is_regular_file(type_ec) || type_ec) {
      continue;
    }
    try {
      out.push_back(p.string());
    } catch (const std::system_error&) {
      // The name has no representation in the narrow (ANSI) encoding, so no
      // std::string-taking API could open it anyway.
      continue;
    }
  }
  if (ec) {
    return unreadable(ec);
  }
  std::sort(out.begin(), out.end());
  return Result<std::vector<std::string>>::Ok(std::move(out));
}

}  // namespace bf
