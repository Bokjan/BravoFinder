// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/bfdb_inventory.h"

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <unordered_map>
#include <utility>
#include <vector>

#include "io/cache/bfdb_naming.h"
#include "io/cache/unified_cache.h"
#include "io/fs_util.h"

namespace bf {

namespace fs = std::filesystem;

Result<BfdbInventory> BfdbInventory::Scan(const std::string& dir) {
  // ListFiles never throws (a range-for over directory_iterator would, on an
  // unreadable entry) and returns the files in a fixed order.
  Result<std::vector<std::string>> files = ListFiles(dir, ".bfdb");
  if (!files) {
    return Result<BfdbInventory>::Err(std::move(files).error());
  }

  BfdbInventory inv;
  // Best cache seen per cycle so far. The header is authoritative for the cycle;
  // ParseBfdbName only gates which files we bother opening.
  std::unordered_map<uint32_t, BfdbEntry> best;
  const auto canonical = [](const BfdbEntry& e) {
    return fs::path(e.path).filename().string() == FormatBfdbName(e.cycle);
  };

  for (const std::string& path : files.value()) {
    // Only "nav_<cycle>.bfdb" names are candidates.
    if (!ParseBfdbName(path)) {
      continue;
    }
    // The filename got us here; the header decides the real cycle.
    Result<UnifiedHeader> header = UnifiedCache::ReadHeader(path);
    if (!header) {
      inv.skipped_.push_back(path);
      continue;
    }
    BfdbEntry entry{path, header.value().cycle};
    auto found = best.find(entry.cycle);
    if (found == best.end()) {
      best.emplace(entry.cycle, entry);
    } else if (canonical(entry) && !canonical(found->second)) {
      // The file named for its own cycle beats a stray copy (a legacy
      // nav.bfdb, a renamed file); otherwise the first in path order stays.
      inv.discarded_.push_back(found->second);
      found->second = entry;
    } else {
      inv.discarded_.push_back(entry);
    }
  }

  inv.entries_.reserve(best.size());
  for (auto& [cycle, entry] : best) {
    inv.entries_.push_back(std::move(entry));
  }
  std::sort(inv.entries_.begin(), inv.entries_.end(),
            [](const BfdbEntry& a, const BfdbEntry& b) { return a.cycle < b.cycle; });
  return Result<BfdbInventory>::Ok(std::move(inv));
}

std::optional<BfdbEntry> BfdbInventory::Latest() const {
  if (entries_.empty()) {
    return std::nullopt;
  }
  // entries_ is sorted by cycle ascending, so the last element is the newest.
  assert(std::is_sorted(entries_.begin(), entries_.end(),
                        [](const BfdbEntry& a, const BfdbEntry& b) { return a.cycle < b.cycle; }));
  return entries_.back();
}

std::optional<BfdbEntry> BfdbInventory::Find(uint32_t cycle) const {
  // entries_ is sorted by cycle ascending, so binary-search for the cycle.
  assert(std::is_sorted(entries_.begin(), entries_.end(),
                        [](const BfdbEntry& a, const BfdbEntry& b) { return a.cycle < b.cycle; }));
  auto it = std::lower_bound(entries_.begin(), entries_.end(), cycle,
                             [](const BfdbEntry& e, uint32_t c) { return e.cycle < c; });
  if (it != entries_.end() && it->cycle == cycle) {
    return *it;
  }
  return std::nullopt;
}

}  // namespace bf
