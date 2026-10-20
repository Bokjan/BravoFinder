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
    std::unique_lock<std::mutex> lock(mutex_);
    // Fast path: already opened. Lock only the map access, not any disk I/O.
    auto it = cache_.find(key);
    if (it != cache_.end()) {
      return Result<const NavDatabase*>::Ok(it->second.get());
    }
    // Coalesce a burst of first requests: the first caller opens, the rest wait
    // here rather than each running (and mostly discarding) a full open. The
    // wait ends when the opener publishes the database -- or the failure, which
    // leaves `opening_` without a cache entry, so a waiter retries it itself.
    if (opening_.contains(key)) {
      opened_cv_.wait(lock, [&] { return !opening_.contains(key); });
      auto ready = cache_.find(key);
      if (ready != cache_.end()) {
        return Result<const NavDatabase*>::Ok(ready->second.get());
      }
    }
    opening_.emplace(key, true);
  }

  // Open outside the lock so concurrent Gets for different cycles parse in
  // parallel. CIFP procedures live in the same unified .bfdb; cifp_load_ selects
  // on-demand (per-airport on first query) or eager (all up front, then
  // lock-free) loading after OpenCached.
  Result<NavDatabase> opened = NavDatabase::OpenCached(entry->path, cifp_load_);

  std::lock_guard<std::mutex> lock(mutex_);
  if (opened) {
    cache_.emplace(key, std::make_unique<NavDatabase>(std::move(opened).value()));
  }
  opening_.erase(key);
  // A failed open is not cached: the next caller retries it (a transient read
  // error should not poison the cycle for the process lifetime).
  opened_cv_.notify_all();
  if (!opened) {
    return Result<const NavDatabase*>::Err(std::move(opened).error());
  }
  return Result<const NavDatabase*>::Ok(cache_.at(key).get());
}

}  // namespace bf::service
