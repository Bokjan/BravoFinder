// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// BfdbInventory: the set of `.bfdb` caches found in a directory, indexed by
// AIRAC cycle, so a server can offer multiple cycles from one folder.
//
// Scan reads each cache's HEADER (authoritative cycle), not its filename, so a
// renamed file is still cataloged correctly. The filename is used only to find
// candidate files (nav_*.bfdb); see bfdb_naming.h.
//
// When several caches share a cycle, the last one scanned wins and the rest are
// recorded in discarded() -- never silently dropped -- so the caller can report
// them. (There is no `build` stamp to rank same-cycle files by anymore.)

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/result.h"

namespace bf {

// One cataloged cache: its path plus the authoritative cycle from the header.
struct BfdbEntry {
  std::string path;
  uint32_t cycle = 0;
};

class BfdbInventory {
 public:
  // Scan `dir` for `nav_*.bfdb` caches, reading each header for its cycle. Files
  // that fail to open or whose header is corrupt/incompatible are skipped and
  // listed in skipped(). Returns an error only when the directory itself cannot
  // be read.
  static Result<BfdbInventory> Scan(const std::string& dir);

  // The winning cache for each cycle, ordered by cycle ascending. Empty when the
  // directory held no valid caches.
  const std::vector<BfdbEntry>& entries() const { return entries_; }

  // Same-cycle caches shadowed by a later-scanned winner, and files skipped for
  // a bad/unreadable header. Non-authoritative; exposed so a caller can warn
  // rather than have coverage silently reduced.
  const std::vector<BfdbEntry>& discarded() const { return discarded_; }
  const std::vector<std::string>& skipped() const { return skipped_; }

  // The newest cache: highest cycle. nullopt when empty.
  std::optional<BfdbEntry> Latest() const;

  // The winning cache for `cycle`, or nullopt if that cycle is not present.
  std::optional<BfdbEntry> Find(uint32_t cycle) const;

  bool empty() const { return entries_.empty(); }

 private:
  std::vector<BfdbEntry> entries_;    // one per cycle, sorted by cycle
  std::vector<BfdbEntry> discarded_;  // shadowed same-cycle caches
  std::vector<std::string> skipped_;  // files with unreadable/incompatible headers
};

}  // namespace bf
