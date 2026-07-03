#pragma once

// NavDatabaseRegistry: serves multiple AIRAC cycles from one directory of
// `.bfdb` caches, opening each cycle's database lazily on first use.
//
// Built from a BfdbInventory (the directory scan). A Get(cycle) call opens and
// caches that cycle's NavDatabase on first access, then returns the cached
// instance; Get() with no cycle serves the newest (latest) cycle. Opening is
// on-demand for CIFP (a server may touch many cycles; eager loading each would
// cost ~100 MB apiece).
//
// Thread-safety: Get is safe to call concurrently. A mutex guards only the map
// lookup/insert, never the disk open, mirroring NavDatabase's procedure cache.
// The cache stores unique_ptr values and is append-only, so a returned
// NavDatabase pointer stays valid for the registry's lifetime even when a
// concurrent insert rehashes the map. Each opened NavDatabase is itself
// read-only per contract B, so concurrent queries across cycles are safe.

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "core/result.h"
#include "io/cache/bfdb_inventory.h"
#include "io/nav_database.h"

namespace bf::mcp {

class NavDatabaseRegistry {
 public:
  // Take ownership of the directory inventory. Databases are opened lazily; the
  // constructor does no I/O beyond what building the inventory already did.
  explicit NavDatabaseRegistry(BfdbInventory inventory);

  // The database for `cycle`, or the latest cycle when nullopt. Opens and
  // caches it on first use. Returns an error if the cycle is unknown, the
  // directory held no caches, or the cache fails to open. The returned pointer
  // is owned by the registry and valid for its lifetime.
  Result<const NavDatabase*> Get(std::optional<uint32_t> cycle);

  // The underlying inventory, for enumerating available cycles (list_cycles).
  const BfdbInventory& inventory() const { return inventory_; }

 private:
  BfdbInventory inventory_;
  // cycle -> opened database. Append-only under mutex_; unique_ptr values keep
  // returned pointers stable across rehash.
  std::unordered_map<uint32_t, std::unique_ptr<NavDatabase>> cache_;
  std::mutex mutex_;
};

}  // namespace bf::mcp
