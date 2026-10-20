// SPDX-License-Identifier: MIT
#pragma once

// NavDatabaseRegistry: serves multiple AIRAC cycles from one directory of
// `.bfdb` caches, opening each cycle's database lazily on first use.
//
// Built from a BfdbInventory (the directory scan). A Get(cycle) call opens and
// caches that cycle's NavDatabase on first access, then returns the cached
// instance; Get() with no cycle serves the newest (latest) cycle. The CIFP load
// mode is chosen once at construction and applies to every cycle: on-demand by
// default (a server may touch many cycles; eager loading each would cost ~100 MB
// apiece), or eager for lock-free procedure reads when the deployment serves a
// small, fixed set of cycles and wants to avoid the per-airport lock/pread.
//
// Thread-safety: Get is safe to call concurrently. A mutex guards only the map
// lookup/insert, never the disk open, mirroring NavDatabase's procedure cache.
// Concurrent first-requests for the same cycle are coalesced: one thread opens,
// the others wait for its result instead of each running a full open.
// The cache stores unique_ptr values and is append-only, so a returned
// NavDatabase pointer stays valid for the registry's lifetime even when a
// concurrent insert rehashes the map. Each opened NavDatabase is itself
// read-only per thread-safety contract, so concurrent queries across cycles are safe.

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "core/result.h"
#include "io/cache/bfdb_inventory.h"
#include "io/navdb/nav_database.h"

namespace bf::service {

class NavDatabaseRegistry {
 public:
  // Take ownership of the directory inventory. Databases are opened lazily; the
  // constructor does no I/O beyond what building the inventory already did.
  // `cifp_load` selects on-demand (default) or eager CIFP loading for every
  // cycle this registry opens (see CifpLoad).
  explicit NavDatabaseRegistry(BfdbInventory inventory, CifpLoad cifp_load = CifpLoad::kOnDemand);

  // The database for `cycle`, or the latest cycle when nullopt. Opens and
  // caches it on first use. Returns an error if the cycle is unknown, the
  // directory held no caches, or the cache fails to open. The returned pointer
  // is owned by the registry and valid for its lifetime.
  Result<const NavDatabase*> Get(std::optional<uint32_t> cycle);

  // The underlying inventory, for enumerating available cycles (list_cycles).
  const BfdbInventory& inventory() const { return inventory_; }

 private:
  BfdbInventory inventory_;
  CifpLoad cifp_load_;  // CIFP load mode applied to every cycle opened here
  // cycle -> opened database. Append-only under mutex_; unique_ptr values keep
  // returned pointers stable across rehash.
  std::unordered_map<uint32_t, std::unique_ptr<NavDatabase>> cache_;
  std::mutex mutex_;
  // Cycles currently being opened, and a wake-up channel for the threads waiting
  // on them. Without this, a burst of first requests for one cycle each ran a
  // full OpenCached (~100 MB apiece when eager) and all but one result was
  // discarded. Only the first caller opens; the rest wait and then find the
  // cached database (or the same failure).
  std::unordered_map<uint32_t, bool> opening_;  // cycle -> true while in progress
  std::condition_variable opened_cv_;
};

}  // namespace bf::service
