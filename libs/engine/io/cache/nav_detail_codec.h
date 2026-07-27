// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>
#include <span>
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

class ByteWriter;  // io/cache/byte_io.h
class StringPool;  // io/cache/byte_io.h

// In-memory view of the navaid-detail section of a unified `.bfdb`: two sorted
// arrays backed by binary search. All data is loaded eagerly at decode time
// (~5 MB).
//
// Thread-safety: immutable after construction; all lookup methods are const and
// share no mutable state, satisfying NavDatabase contract B.
class NavDetailArchive {
 public:
  NavDetailArchive() = default;

  // Build an archive directly from a loader's parsed data (the `bf build` /
  // no-cache `Open()` path): converts NavaidDetail/HoldFix into the query-facing
  // Info types and sorts both arrays for binary search. The resulting archive is
  // ready both for lookups and for NavDetailCodec::Encode to serialize.
  static NavDetailArchive FromData(const NavData& data);

  // Look up all detail records matching `ident` (any region).
  // Returns an empty vector if the archive was not loaded or no match exists.
  std::vector<NavaidDetailInfo> FindNavaids(const std::string& ident) const;

  // Look up all hold patterns for a fix `ident` (any region).
  std::vector<HoldInfo> FindHolds(const std::string& ident) const;

  bool loaded() const { return loaded_; }

 private:
  friend class NavDetailCodec;

  // Sort both arrays and mark the archive loaded. Shared by FromData and Decode.
  void Finalize();

  // Sorted by (ident.ident, ident.region); lower_bound for exact match.
  std::vector<std::pair<Ident, NavaidDetailInfo>> navaids_;
  // Sorted by fix.ident string; equal_range for multi-value lookup.
  std::vector<HoldInfo> holds_;

  bool loaded_ = false;
};

// Encode/decode the navaid-detail SECTION of a unified `.bfdb`: navaid detail
// attributes (freq/range/elev/heading) and holding patterns, indexed for fast
// binary-search lookup. This codec owns only the "struct <-> bytes" mapping for
// the detail payload; it carries no format version of its own (the container
// owns the single version). String references point into the container's global
// pool.
class NavDetailCodec {
 public:
  // Append the detail section body (navaid + hold records) to `w`, interning
  // strings into the shared `pool`.
  static Result<void> Encode(const NavDetailArchive& archive, ByteWriter& w, StringPool& pool);

  // Decode a detail section body (`body`) into a NavDetailArchive, resolving
  // string references against the global pool blob (`pool`).
  static Result<NavDetailArchive> Decode(std::span<const uint8_t> body,
                                         std::span<const uint8_t> pool);
};

}  // namespace bf
