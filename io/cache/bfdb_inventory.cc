#include "io/cache/bfdb_inventory.h"

#include <algorithm>
#include <filesystem>
#include <system_error>
#include <unordered_map>

#include "io/cache/bfdb_cache.h"
#include "io/cache/bfdb_naming.h"

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
  // Best cache seen per cycle so far. The header is authoritative for
  // cycle/build; ParseBfdbName only gates which files we bother opening.
  std::unordered_map<uint32_t, BfdbEntry> best;

  for (const fs::directory_entry& de : it) {
    if (!de.is_regular_file(ec) || ec) {
      continue;
    }
    // Only "nav_<cycle>_<build>.bfdb" names are candidates; ParseBfdbName
    // rejects the "*_cifp.bfdb" companions and anything else.
    const std::string path = de.path().string();
    if (!ParseBfdbName(de.path().filename().string())) {
      continue;
    }
    // The filename got us here; the header decides the real cycle/build.
    Result<BfdbHeader> header = BfdbCache::ReadHeader(path);
    if (!header) {
      inv.skipped_.push_back(path);
      continue;
    }
    BfdbEntry entry{path, header.value().cycle, header.value().build};
    auto found = best.find(entry.cycle);
    if (found == best.end()) {
      best.emplace(entry.cycle, entry);
    } else if (entry.build > found->second.build) {
      inv.discarded_.push_back(found->second);  // the old winner is now shadowed
      found->second = entry;
    } else {
      inv.discarded_.push_back(entry);  // this one loses to the existing winner
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
  // entries_ is sorted by cycle ascending and holds the highest build per
  // cycle, so the last element is the newest overall.
  return entries_.back();
}

std::optional<BfdbEntry> BfdbInventory::Find(uint32_t cycle) const {
  for (const BfdbEntry& e : entries_) {
    if (e.cycle == cycle) {
      return e;
    }
  }
  return std::nullopt;
}

}  // namespace bf
