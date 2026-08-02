// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <algorithm>
#include <string>

namespace bf {

// Upper-case an ASCII string in place and return it. Used to normalize
// user-supplied idents / ICAO codes / airway names before lookup, since the
// navigation data is stored upper-case. ASCII-only by design and genuinely
// locale-independent: only 'a'..'z' are folded (std::toupper would consult the
// current locale), so results are stable across platforms and locales.
inline std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](char c) -> char {
    return (c >= 'a' && c <= 'z') ? static_cast<char>(c - ('a' - 'A')) : c;
  });
  return s;
}

}  // namespace bf
