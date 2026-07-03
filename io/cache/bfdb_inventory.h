#pragma once

// BfdbInventory: the set of `.bfdb` graph caches found in a directory, indexed
// by AIRAC cycle, so a server can offer multiple cycles from one folder.
//
// Scan reads each cache's HEADER (authoritative cycle/build), not its filename,
// so a renamed file is still cataloged correctly. The filename is used only to
// find candidate files (nav_*.bfdb) and skip the *_cifp.bfdb companions; see
// bfdb_naming.h.
//
// When several caches share a cycle (e.g. two builds of the same AIRAC), the
// highest build wins and the rest are recorded in discarded() -- never silently
// dropped -- so the caller can report them.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/result.h"

namespace bf {

// One cataloged cache: its path plus the authoritative cycle/build from the
// header.
struct BfdbEntry {
  std::string path;
  uint32_t cycle = 0;
  uint32_t build = 0;
};

class BfdbInventory {
 public:
  // Scan `dir` for `.bfdb` graph caches (excluding `*_cifp.bfdb` companions),
  // reading each header for its cycle/build. Files that fail to open or whose
  // header is corrupt/incompatible are skipped and listed in skipped(). Returns
  // an error only when the directory itself cannot be read.
  static Result<BfdbInventory> Scan(const std::string& dir);

  // The winning cache for each cycle (highest build), ordered by cycle
  // ascending. Empty when the directory held no valid caches.
  const std::vector<BfdbEntry>& entries() const { return entries_; }

  // Lower-build caches shadowed by a same-cycle winner, and files skipped for a
  // bad/unreadable header. Non-authoritative; exposed so a caller can warn
  // rather than have coverage silently reduced.
  const std::vector<BfdbEntry>& discarded() const { return discarded_; }
  const std::vector<std::string>& skipped() const { return skipped_; }

  // The newest cache: highest cycle, then highest build. nullopt when empty.
  std::optional<BfdbEntry> Latest() const;

  // The winning cache for `cycle`, or nullopt if that cycle is not present.
  std::optional<BfdbEntry> Find(uint32_t cycle) const;

  bool empty() const { return entries_.empty(); }

 private:
  std::vector<BfdbEntry> entries_;    // one per cycle, sorted by cycle
  std::vector<BfdbEntry> discarded_;  // shadowed lower builds
  std::vector<std::string> skipped_;  // files with unreadable/incompatible headers
};

}  // namespace bf
