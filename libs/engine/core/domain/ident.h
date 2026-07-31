// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <utility>

namespace bf {

// A navigation point identifier. An ident string (e.g. "JFK", "DEEZZ") is not
// globally unique: the same code is reused across ICAO regions. The (ident,
// arinc424_icao_code) pair is what uniquely identifies a point, so it is modeled
// as one value type and used as the lookup key throughout.
//
// `arinc424_icao_code` is the two-letter ICAO region indicator defined by
// ARINC 424 §5.6 ("ICAO Code", aligned with ICAO Annex 10), e.g. "K6", "ZB",
// "LF". It is NOT the airport ICAO 4-letter identifier (e.g. "KJFK") -- that
// lives in Airport::icao. The "arinc424_" prefix pins the standard so the two
// cannot be confused.
//
// Source column names differ per loader but the value is always the ARINC 424
// ICAO Code: DFD/Fenix SQLite names it `icao_code`, X-Plane earth_*.dat names it
// `region`, Fenix WaypointLookup names it `Country`. Do NOT confuse it with the
// DFD/Fenix `region_code` SQL column, which is the 4-5-char airport id a
// terminal fix belongs to (e.g. "01OH") and is NOT a region -- that mismatched
// name is what caused the Fenix holding bug.
struct Ident {
  std::string ident{};
  std::string arinc424_icao_code{};

  Ident() = default;
  Ident(std::string id, std::string arinc424_icao_code)
      : ident(std::move(id)), arinc424_icao_code(std::move(arinc424_icao_code)) {}

  bool operator==(const Ident& other) const {
    return ident == other.ident && arinc424_icao_code == other.arinc424_icao_code;
  }
};

}  // namespace bf

namespace std {

template <>
struct hash<bf::Ident> {
  size_t operator()(const bf::Ident& key) const noexcept {
    size_t h1 = std::hash<std::string>{}(key.ident);
    size_t h2 = std::hash<std::string>{}(key.arinc424_icao_code);
    // Combine the two hashes (boost-style mix).
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

}  // namespace std
