// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "core/base/attributes.h"
#include "core/domain/ident.h"

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
// corrupt source data and trips an assert (debug) rather than truncating.
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

  uint8_t ident_len = 0;
  uint8_t arinc424_icao_code_len = 0;
  // Length-prefixed, NOT NUL-terminated: all cap bytes hold characters, so there
  // is no room for a terminator and `ident`/`arinc424_icao_code` are NOT C
  // strings. Read them only via IdentView()/Arinc424IcaoCodeView() or the raw
  // pointer + its _len. This keeps the struct at exactly 12 bytes (owner chose
  // 12 over 16). Consistent with FixedName, which is length-prefixed for the
  // same reason.
  char ident[kIdentCap] = {};                          // 7 bytes
  char arinc424_icao_code[kArinc424IcaoCodeCap] = {};  // 3 bytes
  // 1 + 1 + 7 + 3 = 12; alignas(1) leaves no padding.

  std::string_view IdentView() const BF_LIFETIMEBOUND { return {ident, ident_len}; }
  std::string_view Arinc424IcaoCodeView() const BF_LIFETIMEBOUND {
    return {arinc424_icao_code, arinc424_icao_code_len};
  }

  bool operator==(const FixedIdent& o) const noexcept {
    return ident_len == o.ident_len && arinc424_icao_code_len == o.arinc424_icao_code_len &&
           std::memcmp(ident, o.ident, ident_len) == 0 &&
           std::memcmp(arinc424_icao_code, o.arinc424_icao_code, arinc424_icao_code_len) == 0;
  }

  // Order by (ident, arinc424_icao_code) for sorted-vector indices + binary search.
  bool operator<(const FixedIdent& o) const noexcept {
    const int c = IdentView().compare(o.IdentView());
    return c != 0 ? c < 0 : Arinc424IcaoCodeView() < o.Arinc424IcaoCodeView();
  }

  // Pack an Ident into the fixed form. Overflow (a field longer than its cap)
  // asserts in debug and is impossible on real data; in release the copy is
  // clamped to the cap so a corrupt oversized field cannot overrun the buffer.
  static FixedIdent FromIdent(const Ident& id) {
    return FromParts(id.ident, id.arinc424_icao_code);
  }

  static FixedIdent FromParts(std::string_view id, std::string_view arinc424_icao_code) {
    assert(id.size() <= kIdentCap && "ident overflows FixedIdent::kIdentCap");
    assert(arinc424_icao_code.size() <= kArinc424IcaoCodeCap &&
           "arinc424_icao_code overflows FixedIdent::kArinc424IcaoCodeCap");
    FixedIdent f;
    f.ident_len = static_cast<uint8_t>(id.size() < kIdentCap ? id.size() : kIdentCap);
    f.arinc424_icao_code_len = static_cast<uint8_t>(arinc424_icao_code.size() < kArinc424IcaoCodeCap
                                                        ? arinc424_icao_code.size()
                                                        : kArinc424IcaoCodeCap);
    std::memcpy(f.ident, id.data(), f.ident_len);
    std::memcpy(f.arinc424_icao_code, arinc424_icao_code.data(), f.arinc424_icao_code_len);
    return f;
  }

  // Materialize back to an owned Ident. Both fields fit libstdc++'s SSO (15B),
  // so this allocates nothing on the heap.
  Ident ToIdent() const {
    return Ident(std::string(IdentView()), std::string(Arinc424IcaoCodeView()));
  }
};

static_assert(sizeof(FixedIdent) == 12, "FixedIdent must stay 12 bytes");

// A length-prefixed, region-less short name of exactly N bytes (1 length byte +
// N-1 chars). Used for sorted-vector lookup keys and compact query fields that
// carry no region: bare fix idents / AirwayLeg endpoints, airport ICAOs
// (airport_index_ / CifpArchive directory), airway designators (airway_index_).
//
// N is the total sizeof, not the character capacity: FixedName<8> is 8 bytes
// with kCap=7. Keeping sizeof(key)<=8 makes pair<FixedName8,int> stay at 12
// bytes (a 9..12B key would pad to 16B). Real AIRAC names are <=5 chars, so
// the length byte's cost in capacity is fine.
//
// Length-prefixed (not NUL-terminated / zero-padded): View() is O(1) and
// ordering is memcmp of the common prefix then length -- no strnlen scan.
template <int N>
struct alignas(1) FixedName {
  static_assert(N >= 2, "FixedName needs a length byte + at least 1 char");

  static constexpr int kSize = N;
  static constexpr int kCap = N - 1;  // chars after the length byte

  uint8_t len = 0;
  // Length-prefixed, NOT NUL-terminated: all kCap bytes hold characters, so
  // there is NO room for a terminator and `text` is NOT a C string. Never pass
  // it to strlen/strcmp/printf("%s")/any <cstring> C-string API or treat it as
  // null-terminated. Read it only via View() (length-bounded) or text + len.
  char text[kCap] = {};

  std::string_view View() const BF_LIFETIMEBOUND { return {text, len}; }

  // Ordering for sorted-array + binary search: common-prefix memcmp then length.
  bool operator<(const FixedName& o) const {
    const int c = std::memcmp(text, o.text, std::min(len, o.len));
    return c != 0 ? c < 0 : len < o.len;
  }

  bool operator==(const FixedName& o) const {
    return len == o.len && std::memcmp(text, o.text, len) == 0;
  }

  static FixedName From(std::string_view s) {
    assert(s.size() <= kCap && "name overflows FixedName::kCap");
    FixedName f;
    f.len = static_cast<uint8_t>(s.size() < kCap ? s.size() : kCap);
    std::memcpy(f.text, s.data(), f.len);
    return f;
  }
};

static_assert(sizeof(FixedName<8>) == 8, "FixedName<8> must stay 8 bytes");

using FixedName8 = FixedName<8>;

}  // namespace bf
