#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/domain/hold_fix.h"
#include "core/domain/ident.h"
#include "core/domain/navaid_detail.h"
#include "core/query/query_types.h"
#include "core/result.h"
#include "io/nav_data.h"

namespace bf {

// In-memory view of a nav_detail.bfdb file after Open(): two sorted arrays
// backed by binary search. All data is loaded eagerly at Open time (~5 MB).
//
// Thread-safety: immutable after Open(); all lookup methods are const and
// share no mutable state, satisfying NavDatabase contract B.
class NavDetailArchive {
 public:
  NavDetailArchive() = default;

  // Build an archive directly from a loader's parsed data (the `bf build` /
  // no-cache `Open()` path): converts NavaidDetail/HoldFix into the query-facing
  // Info types and sorts both arrays for binary search. The resulting archive is
  // ready both for lookups and for NavDetailCache::Build to serialize.
  static NavDetailArchive FromData(const NavData& data);

  // Look up all detail records matching `ident` (any region).
  // Returns an empty vector if the archive was not loaded or no match exists.
  std::vector<NavaidDetailInfo> FindNavaids(const std::string& ident) const;

  // Look up all hold patterns for a fix `ident` (any region).
  std::vector<HoldInfo> FindHolds(const std::string& ident) const;

  bool loaded() const { return loaded_; }
  uint32_t cycle() const { return cycle_; }
  uint32_t build() const { return build_; }

 private:
  friend class NavDetailCache;

  // Sort both arrays and mark the archive loaded. Shared by FromData and Open.
  void Finalize();

  // Sorted by (ident.ident, ident.region); lower_bound for exact match.
  std::vector<std::pair<Ident, NavaidDetailInfo>> navaids_;
  // Sorted by fix.ident string; equal_range for multi-value lookup.
  std::vector<HoldInfo> holds_;

  bool loaded_ = false;
  uint32_t cycle_ = 0;
  uint32_t build_ = 0;
};

// Builds and opens the nav_<cycle>_<build>_detail.bfdb side-cache: navaid
// detail attributes (freq/range/elev/heading) and holding patterns, indexed
// for fast binary-search lookup. Same little-endian format family as bfdb/cifp.
class NavDetailCache {
 public:
  // v1: initial format (magic "BFND").
  static constexpr uint32_t kFormatVersion = 1;

  // Serialize a built archive into `out_path`, recording the given provenance in
  // the header. Source-agnostic: takes an already-built archive, not raw data.
  static Result<void> Build(const std::string& out_path, const NavDetailArchive& archive,
                            const std::string& source_loader, const std::string& program_semver);

  // Read the entire file into memory and build lookup indices.
  // Returns kDataMissing if the file is absent or has an incompatible format.
  static Result<NavDetailArchive> Open(const std::string& path);
};

}  // namespace bf
