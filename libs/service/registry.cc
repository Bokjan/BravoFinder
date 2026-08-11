// SPDX-License-Identifier: MIT
#include "registry.h"

#include <format>
#include <utility>

namespace bf::service {

NavDatabaseRegistry::NavDatabaseRegistry(BfdbInventory inventory, CifpLoad cifp_load)
    : inventory_(std::move(inventory)), cifp_load_(cifp_load) {}

Result<const NavDatabase*> NavDatabaseRegistry::Get(std::optional<uint32_t> cycle) {
  // Resolve which cache to serve: an explicit cycle, else the newest one.
  std::optional<BfdbEntry> entry = cycle ? inventory_.Find(*cycle) : inventory_.Latest();
  if (!entry) {
    // Never dereference an empty optional: an explicit cycle that is missing is
    // "unknown cycle"; a missing Latest() (no cycle asked) means nothing loaded.
    if (cycle) {
      return Result<const NavDatabase*>::Err(
          Error(ErrorCode::kDataMissing, std::format("unknown AIRAC cycle: {}", *cycle)));
    }
    return Result<const NavDatabase*>::Err(
        Error(ErrorCode::kDataMissing, "no navigation databases loaded"));
  }

  const uint32_t key = entry->cycle;
  {
    // Fast path: already opened. Lock only the map access, not any disk I/O.
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      return Result<const NavDatabase*>::Ok(it->second.get());
    }
  }

  // Open outside the lock so concurrent Gets for different cycles parse in
  // parallel. CIFP procedures live in the same unified .bfdb; cifp_load_ selects
  // on-demand (per-airport on first query) or eager (all up front, then
  // lock-free) loading after OpenCached.
  Result<NavDatabase> opened = NavDatabase::OpenCached(entry->path, cifp_load_);
  if (!opened) {
    return Result<const NavDatabase*>::Err(std::move(opened).error());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // Another thread may have opened the same cycle while we parsed; keep the
  // first winner so a returned pointer is never invalidated.
  auto it = cache_.find(key);
  if (it == cache_.end()) {
    it = cache_.emplace(key, std::make_unique<NavDatabase>(std::move(opened).value())).first;
  }
  return Result<const NavDatabase*>::Ok(it->second.get());
}

}  // namespace bf::service
