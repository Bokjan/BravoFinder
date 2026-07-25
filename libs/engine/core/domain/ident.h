// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <functional>
#include <string>
#include <string_view>
#include <utility>

namespace bf {

// A navigation point identifier. An ident string (e.g. "JFK", "DEEZZ") is not
// globally unique: the same code is reused across ICAO regions. The (ident,
// region) pair is what uniquely identifies a point, so it is modeled as one
// value type and used as the lookup key throughout.
struct Ident {
  std::string ident{};
  std::string region{};  // two-letter ICAO region code, e.g. "K6"

  Ident() = default;
  Ident(std::string id, std::string reg) : ident(std::move(id)), region(std::move(reg)) {}

  bool operator==(const Ident& other) const {
    return ident == other.ident && region == other.region;
  }
};

}  // namespace bf

namespace std {

template <>
struct hash<bf::Ident> {
  size_t operator()(const bf::Ident& key) const noexcept {
    size_t h1 = std::hash<std::string>{}(key.ident);
    size_t h2 = std::hash<std::string>{}(key.region);
    // Combine the two hashes (boost-style mix).
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

}  // namespace std
