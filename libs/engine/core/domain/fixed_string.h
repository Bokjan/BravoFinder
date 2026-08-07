// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <format>
#include <string_view>

#include "core/base/attributes.h"
#include "core/domain/ident.h"
#include "core/result.h"

namespace bf {

// A fixed-size, inline-stored counterpart to Ident, used only for the large
// per-vertex idents_ array (size = V ~= 270k). Ident is 2x std::string (64B on
// libstdc++); FixedIdent packs the same (ident, arinc424_icao_code) into 12
// bytes, cutting that array from ~17 MB to ~3 MB with no heap allocation per
// entry.
//
// Ident remains the domain-edge type wherever elasticity or owned storage is
// needed (NavData, ProcedureLeg, query results). FixedIdent is purely the
// compact storage form; the two convert at the idents_ boundary via
// FromIdent / ToIdent.
//
// Capacities are sized from real AIRAC data (cycle 2601): fix/nav idents max 5
// chars, ICAO region codes max 2. The caps below carry margin; overflow means
// corrupt source data and is reported as an error rather than truncating.
//
// Layout: character payloads first (so `ident` starts at offset 0 / natural
// alignment within an aligned element), trailing length bytes. NOT
// NUL-terminated — all cap bytes hold characters; read only via IdentView() /
// Arinc424IcaoCodeView(). Fields are private so the only writer is From*;
// unused trailing payload bytes stay zero from value-init.
//
// Naming: the field is `arinc424_icao_code` -- the two-letter ICAO region
// indicator per ARINC 424 §5.6 ("ICAO Code"), e.g. "K6". It is NOT the airport
// ICAO 4-letter identifier. Source columns differ per loader (DFD/Fenix
// `icao_code`, X-Plane `region`, Fenix `Country`) but the value is always the
// ARINC 424 ICAO Code; do not confuse it with the DFD/Fenix `region_code`
// column (the airport id, e.g. "01OH", which is NOT a region).
struct alignas(1) FixedIdent {
  static constexpr int kIdentCap = 7;             // observed max 5, margin to 7
  static constexpr int kArinc424IcaoCodeCap = 3;  // observed max 2, margin to 3

  FixedIdent() = default;

  std::string_view IdentView() const BF_LIFETIMEBOUND { return {ident_, ident_len_}; }
  std::string_view Arinc424IcaoCodeView() const BF_LIFETIMEBOUND {
    return {arinc424_icao_code_, arinc424_icao_code_len_};
  }

  bool operator==(const FixedIdent& o) const noexcept {
    return ident_len_ == o.ident_len_ && arinc424_icao_code_len_ == o.arinc424_icao_code_len_ &&
           std::memcmp(ident_, o.ident_, ident_len_) == 0 &&
           std::memcmp(arinc424_icao_code_, o.arinc424_icao_code_, arinc424_icao_code_len_) == 0;
  }

  // Order by (ident, arinc424_icao_code) for sorted-vector indices + binary search.
  bool operator<(const FixedIdent& o) const noexcept {
    const int c = std::memcmp(ident_, o.ident_, std::min(ident_len_, o.ident_len_));
    if (c != 0) {
      return c < 0;
    }
    if (ident_len_ != o.ident_len_) {
      return ident_len_ < o.ident_len_;
    }
    const int c2 = std::memcmp(arinc424_icao_code_, o.arinc424_icao_code_,
                               std::min(arinc424_icao_code_len_, o.arinc424_icao_code_len_));
    return c2 != 0 ? c2 < 0 : arinc424_icao_code_len_ < o.arinc424_icao_code_len_;
  }

  // Pack an Ident into the fixed form. Overflow (a field longer than its cap)
  // is an error; callers must choose whether their context should reject or
  // skip the invalid source record.
  static Result<FixedIdent> FromIdent(const Ident& id) {
    return FromParts(id.ident, id.arinc424_icao_code);
  }

  static Result<FixedIdent> FromParts(std::string_view id, std::string_view arinc424_icao_code) {
    if (id.size() > kIdentCap) {
      return Result<FixedIdent>::Err(
          Error(ErrorCode::kInvalidArgument,
                std::format("ident '{}' exceeds FixedIdent::kIdentCap ({} > {})", id, id.size(),
                            kIdentCap)));
    }
    if (arinc424_icao_code.size() > kArinc424IcaoCodeCap) {
      return Result<FixedIdent>::Err(
          Error(ErrorCode::kInvalidArgument,
                std::format("ARINC 424 ICAO code '{}' exceeds FixedIdent::kArinc424IcaoCodeCap "
                            "({} > {})",
                            arinc424_icao_code, arinc424_icao_code.size(), kArinc424IcaoCodeCap)));
    }
    FixedIdent f;
    f.ident_len_ = static_cast<uint8_t>(id.size());
    f.arinc424_icao_code_len_ = static_cast<uint8_t>(arinc424_icao_code.size());
    std::memcpy(f.ident_, id.data(), f.ident_len_);
    std::memcpy(f.arinc424_icao_code_, arinc424_icao_code.data(), f.arinc424_icao_code_len_);
    return Result<FixedIdent>::Ok(std::move(f));
  }

  // Materialize back to an owned Ident. Both fields fit libstdc++'s SSO (15B),
  // so this allocates nothing on the heap.
  Ident ToIdent() const {
    return Ident(std::string(IdentView()), std::string(Arinc424IcaoCodeView()));
  }

 private:
  // 7 + 3 + 1 + 1 = 12; alignas(1) leaves no padding.
  char ident_[kIdentCap] = {};
  char arinc424_icao_code_[kArinc424IcaoCodeCap] = {};
  uint8_t ident_len_ = 0;
  uint8_t arinc424_icao_code_len_ = 0;
};

static_assert(sizeof(FixedIdent) == 12, "FixedIdent must stay 12 bytes");

// A short name of exactly N bytes (N-1 chars + 1 trailing length byte). Used
// for sorted-vector lookup keys and compact query fields that carry no region:
// bare fix idents / AirwayLeg endpoints, airport ICAOs (airport_index_ /
// CifpArchive directory), airway designators (airway_index_).
//
// N is the total sizeof, not the character capacity: FixedName<8> is 8 bytes
// with kCap=7. Keeping sizeof(key)<=8 makes pair<FixedName8,int> stay at 12
// bytes (a 9..12B key would pad to 16B). Real AIRAC names are <=5 chars, so
// the length byte's cost in capacity is fine.
//
// Payload-first + trailing length (not NUL-terminated): `text` starts at
// offset 0; View() is O(1); ordering is memcmp of the common prefix then
// length. Fields are private; only From writes.
template <int N>
struct alignas(1) FixedName {
  static_assert(N >= 2, "FixedName needs at least 1 char + a length byte");

  static constexpr int kSize = N;
  static constexpr int kCap = N - 1;  // chars before the trailing length byte

  FixedName() = default;

  std::string_view View() const BF_LIFETIMEBOUND { return {text_, len_}; }

  // Ordering for sorted-array + binary search: common-prefix memcmp then length.
  bool operator<(const FixedName& o) const {
    const int c = std::memcmp(text_, o.text_, std::min(len_, o.len_));
    return c != 0 ? c < 0 : len_ < o.len_;
  }

  bool operator==(const FixedName& o) const {
    return len_ == o.len_ && std::memcmp(text_, o.text_, len_) == 0;
  }

  // Pack a short lookup key. Overflow is an error rather than truncation.
  static Result<FixedName> From(std::string_view s) {
    if (s.size() > kCap) {
      return Result<FixedName>::Err(
          Error(ErrorCode::kInvalidArgument,
                std::format("name '{}' exceeds FixedName::kCap ({} > {})", s, s.size(), kCap)));
    }
    FixedName f;
    f.len_ = static_cast<uint8_t>(s.size());
    std::memcpy(f.text_, s.data(), f.len_);
    return Result<FixedName>::Ok(std::move(f));
  }

 private:
  char text_[kCap] = {};
  uint8_t len_ = 0;
};

static_assert(sizeof(FixedName<8>) == 8, "FixedName<8> must stay 8 bytes");

using FixedName8 = FixedName<8>;

}  // namespace bf
