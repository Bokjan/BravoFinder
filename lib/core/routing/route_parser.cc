#include "core/routing/route_parser.h"

#include <cctype>
#include <sstream>

namespace bf {

std::vector<std::string> TokenizeRoute(const std::string& route_str) {
  std::vector<std::string> tokens;
  std::istringstream stream(route_str);
  std::string tok;
  while (stream >> tok) {  // operator>> skips and collapses all whitespace
    for (char& c : tok) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    tokens.push_back(std::move(tok));
  }
  return tokens;
}

}  // namespace bf
