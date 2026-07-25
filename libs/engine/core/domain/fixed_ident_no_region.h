// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "core/util/attributes.h"

namespace bf {

// An 8-byte, region-less counterpart to FixedIdent, for lookup keys that carry
// no region: a bare fix ident (ident_all_) or an airport ICAO (airport_index_).
// Backs sorted-vector indices, where sizeof==8 keeps pair<FixedIdentNoRegion,int>
// at 12 bytes (a 9..12B key would pad up to 16B, saving nothing -- see
// .notes/plans/2026-07-09_memory_compaction.md #3).
//
// Capacities are sized from real AIRAC data (cycle 2601): fix idents max 5,
// airport ICAO max 4. Overflow means corrupt source data and trips an assert
// (debug); release clamps to the cap so a bad oversized field cannot overrun.
struct alignas(1) FixedIdentNoRegion {
  static constexpr int kCap = 7;  // fix ident max 5 / airport ICAO max 4, margin to 7

  uint8_t len = 0;
  // Length-prefixed, NOT NUL-terminated: all kCap bytes hold characters, so
  // there is NO room for a terminator and `text` is NOT a C string. Never pass
  // it to strlen/strcmp/printf("%s")/any <cstring> C-string API or treat it as
  // null-terminated. Read it only via View() (length-bounded) or text + len.
  char text[kCap] = {};

  std::string_view View() const BF_LIFETIMEBOUND { return {text, len}; }

  // Ordering for sorted-array + binary search: length + memcmp, no strnlen scan.
  bool operator<(const FixedIdentNoRegion& o) const {
    const int c = std::memcmp(text, o.text, std::min(len, o.len));
    return c != 0 ? c < 0 : len < o.len;
  }

  bool operator==(const FixedIdentNoRegion& o) const {
    return len == o.len && std::memcmp(text, o.text, len) == 0;
  }

  static FixedIdentNoRegion From(std::string_view s) {
    assert(s.size() <= kCap && "name overflows FixedIdentNoRegion::kCap");
    FixedIdentNoRegion f;
    f.len = static_cast<uint8_t>(s.size() < kCap ? s.size() : kCap);
    std::memcpy(f.text, s.data(), f.len);
    return f;
  }
};

static_assert(sizeof(FixedIdentNoRegion) == 8, "FixedIdentNoRegion must stay 8 bytes");

}  // namespace bf
