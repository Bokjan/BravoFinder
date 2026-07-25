// SPDX-License-Identifier: LGPL-3.0-or-later
#include "core/routing/route_parser.h"

#include <cctype>
#include <string_view>

namespace bf {

std::vector<std::string> TokenizeRoute(const std::string& route_str) {
  std::vector<std::string> tokens;
  const std::string_view sv(route_str);
  auto is_space = [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; };
  size_t i = 0;
  while (i < sv.size()) {
    while (i < sv.size() && is_space(sv[i])) {  // skip run of whitespace
      ++i;
    }
    const size_t start = i;
    while (i < sv.size() && !is_space(sv[i])) {  // consume the token
      ++i;
    }
    if (i > start) {
      std::string tok(sv.substr(start, i - start));
      for (char& c : tok) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      }
      tokens.push_back(std::move(tok));
    }
  }
  return tokens;
}

}  // namespace bf
