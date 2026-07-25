// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "core/domain/ident.h"
#include "core/util/attributes.h"

namespace bf {

// A fixed-size, inline-stored counterpart to Ident, used only for the large
// per-vertex idents_ array (size = V ~= 270k). Ident is 2x std::string (64B on
// libstdc++); FixedIdent packs the same (ident, region) into 12 bytes, cutting
// that array from ~17 MB to ~3 MB with no heap allocation per entry.
//
// Ident remains the domain-edge type wherever elasticity or owned storage is
// needed (NavData, ProcedureLeg, query results). FixedIdent is purely the
// compact storage form; the two convert at the idents_ boundary via
// FromIdent / ToIdent.
//
// Capacities are sized from real AIRAC data (cycle 2601): fix/nav idents max 5
// chars, ICAO region codes max 2. The caps below carry margin; overflow means
// corrupt source data and trips an assert (debug) rather than truncating.
struct alignas(1) FixedIdent {
  static constexpr int kIdentCap = 7;   // observed max 5, margin to 7
  static constexpr int kRegionCap = 3;  // observed max 2, margin to 3

  uint8_t ident_len = 0;
  uint8_t region_len = 0;
  // Length-prefixed, NOT NUL-terminated: all cap bytes hold characters, so there
  // is no room for a terminator and `ident`/`region` are NOT C strings. Read them
  // only via IdentView()/RegionView() or the raw pointer + its _len. This keeps
  // the struct at exactly 12 bytes (owner chose 12 over 16). Consistent with
  // FixedIdentNoRegion, which is length-prefixed for the same reason.
  char ident[kIdentCap] = {};    // 7 bytes
  char region[kRegionCap] = {};  // 3 bytes
  // 1 + 1 + 7 + 3 = 12; alignas(1) leaves no padding.

  std::string_view IdentView() const BF_LIFETIMEBOUND { return {ident, ident_len}; }
  std::string_view RegionView() const BF_LIFETIMEBOUND { return {region, region_len}; }

  bool operator==(const FixedIdent& o) const {
    return ident_len == o.ident_len && region_len == o.region_len &&
           std::memcmp(ident, o.ident, ident_len) == 0 &&
           std::memcmp(region, o.region, region_len) == 0;
  }

  // Order by (ident, region) for sorted-vector indices + binary search.
  bool operator<(const FixedIdent& o) const {
    const int c = IdentView().compare(o.IdentView());
    return c != 0 ? c < 0 : RegionView() < o.RegionView();
  }

  // Pack an Ident into the fixed form. Overflow (a field longer than its cap)
  // asserts in debug and is impossible on real data; in release the copy is
  // clamped to the cap so a corrupt oversized field cannot overrun the buffer.
  static FixedIdent FromIdent(const Ident& id) { return FromParts(id.ident, id.region); }

  static FixedIdent FromParts(std::string_view id, std::string_view reg) {
    assert(id.size() <= kIdentCap && "ident overflows FixedIdent::kIdentCap");
    assert(reg.size() <= kRegionCap && "region overflows FixedIdent::kRegionCap");
    FixedIdent f;
    f.ident_len = static_cast<uint8_t>(id.size() < kIdentCap ? id.size() : kIdentCap);
    f.region_len = static_cast<uint8_t>(reg.size() < kRegionCap ? reg.size() : kRegionCap);
    std::memcpy(f.ident, id.data(), f.ident_len);
    std::memcpy(f.region, reg.data(), f.region_len);
    return f;
  }

  // Materialize back to an owned Ident. Both fields fit libstdc++'s SSO (15B),
  // so this allocates nothing on the heap.
  Ident ToIdent() const { return Ident(std::string(IdentView()), std::string(RegionView())); }
};

static_assert(sizeof(FixedIdent) == 12, "FixedIdent must stay 12 bytes");

}  // namespace bf
