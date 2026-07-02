#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace bf {

// Upper-case an ASCII string in place and return it. Used to normalize
// user-supplied idents / ICAO codes / airway names before lookup, since the
// navigation data is stored upper-case. ASCII-only by design: navigation
// identifiers are ASCII, and a locale-independent transform keeps results
// stable across platforms.
inline std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

}  // namespace bf
