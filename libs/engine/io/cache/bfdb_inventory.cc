// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/bfdb_inventory.h"

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <system_error>
#include <unordered_map>

#include "io/cache/bfdb_naming.h"
#include "io/cache/unified_cache.h"

namespace bf {

namespace fs = std::filesystem;

Result<BfdbInventory> BfdbInventory::Scan(const std::string& dir) {
  std::error_code ec;
  fs::directory_iterator it(dir, ec);
  if (ec) {
    return Result<BfdbInventory>::Err(Error(
        ErrorCode::kDataMissing, "cannot read directory: " + dir + " (" + ec.message() + ")"));
  }

  BfdbInventory inv;
  // Best cache seen per cycle so far. The header is authoritative for the cycle;
  // ParseBfdbName only gates which files we bother opening.
  std::unordered_map<uint32_t, BfdbEntry> best;

  for (const fs::directory_entry& de : it) {
    if (!de.is_regular_file(ec) || ec) {
      continue;
    }
    // Only "nav_<cycle>.bfdb" names are candidates.
    const std::string path = de.path().string();
    if (!ParseBfdbName(de.path().filename().string())) {
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
    } else {
      // No build stamp to rank same-cycle files by: the later-scanned file wins
      // and the previous one is surfaced in discarded() rather than dropped.
      inv.discarded_.push_back(found->second);
      found->second = entry;
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
