// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "core/result.h"

namespace bf {

// The regular files directly inside `dir` whose extension is `ext` (with the
// dot, e.g. ".dat"), as path strings sorted ascending -- a fixed order, not the
// filesystem's, so a caller picking the first match picks the same file on
// every machine.
//
// Never throws. A range-for over std::filesystem::directory_iterator advances
// with the throwing operator++, and directory_entry::is_regular_file() without
// an error_code throws when a symlink target cannot be stat'ed (EACCES, ELOOP);
// either would terminate the CLI. Here an entry whose type cannot be read, or
// whose name has no narrow-string form (possible on Windows), is skipped, and a
// directory that cannot be opened or read to the end is an Err.
Result<std::vector<std::string>> ListFiles(const std::string& dir, std::string_view ext);

}  // namespace bf
