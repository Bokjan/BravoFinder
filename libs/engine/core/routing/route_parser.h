// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

namespace bf {

// Split a filed-flight-plan route string into whitespace-delimited tokens,
// upper-cased. Handles the common separators (spaces, tabs) and collapses runs
// of whitespace. This is the pure lexical step of ParseRoute; classifying each
// token (airport / airway / SID/STAR / DCT / waypoint) and resolving it on the
// graph is done by NavDatabase::ParseRoute, which has the navigation data.
//
// Example: "KJFK SID CANDR J60 PSB KLAX" -> {KJFK, SID, CANDR, J60, PSB, KLAX}.
std::vector<std::string> TokenizeRoute(const std::string& route_str);

}  // namespace bf
