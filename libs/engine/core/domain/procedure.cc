// SPDX-License-Identifier: LGPL-3.0-or-later
#include "core/domain/procedure.h"

#include <cctype>
#include <charconv>
#include <string>

#include "core/domain/nav_tokens.h"

namespace bf {

bool TerminatesAtFix(PathTerminator t) {
  switch (t) {
    case PathTerminator::kTF:
    case PathTerminator::kIF:
    case PathTerminator::kDF:
    case PathTerminator::kCF:
    case PathTerminator::kAF:  // Arc to Fix: ends at a named fix
    case PathTerminator::kRF:  // Radius to Fix: ends at a named fix
    case PathTerminator::kHF:  // Hold to Fix: ends at a named fix
      return true;
    default:
      return false;
  }
}

AltitudeConstraint ParseAltConstraint(std::string_view desc, int alt1, int alt2) {
  AltitudeConstraint ac;
  if (desc == "+") {
    ac.kind = AltConstraintKind::kAtOrAbove;
    ac.alt1_ft = alt1;
  } else if (desc == "-") {
    ac.kind = AltConstraintKind::kAtOrBelow;
    ac.alt1_ft = alt1;
  } else if (desc == "B") {
    ac.kind = AltConstraintKind::kBetween;
    ac.alt1_ft = alt1;  // upper
    ac.alt2_ft = alt2;  // lower
  } else if (alt1 != 0) {
    // '@' or blank descriptor with an altitude present means "cross at".
    ac.kind = AltConstraintKind::kAt;
    ac.alt1_ft = alt1;
  }
  return ac;
}

PathTerminator ParsePathTerminator(std::string_view token) {
  if (token == "TF") {
    return PathTerminator::kTF;
  }
  if (token == "IF") {
    return PathTerminator::kIF;
  }
  if (token == "DF") {
    return PathTerminator::kDF;
  }
  if (token == "CF") {
    return PathTerminator::kCF;
  }
  if (token == "AF") {
    return PathTerminator::kAF;
  }
  if (token == "RF") {
    return PathTerminator::kRF;
  }
  if (token == "CA") {
    return PathTerminator::kCA;
  }
  if (token == "FA") {
    return PathTerminator::kFA;
  }
  if (token == "VA") {
    return PathTerminator::kVA;
  }
  if (token == "HA") {
    return PathTerminator::kHA;
  }
  if (token == "CD") {
    return PathTerminator::kCD;
  }
  if (token == "FD") {
    return PathTerminator::kFD;
  }
  if (token == "VD") {
    return PathTerminator::kVD;
  }
  if (token == "CI") {
    return PathTerminator::kCI;
  }
  if (token == "VI") {
    return PathTerminator::kVI;
  }
  if (token == "CR") {
    return PathTerminator::kCR;
  }
  if (token == "VR") {
    return PathTerminator::kVR;
  }
  if (token == "FC") {
    return PathTerminator::kFC;
  }
  if (token == "FM") {
    return PathTerminator::kFM;
  }
  if (token == "VM") {
    return PathTerminator::kVM;
  }
  if (token == "PI") {
    return PathTerminator::kPI;
  }
  if (token == "HM") {
    return PathTerminator::kHM;
  }
  if (token == "HF") {
    return PathTerminator::kHF;
  }
  return PathTerminator::kUnknown;
}

std::string PathTerminatorName(PathTerminator t) {
  switch (t) {
    case PathTerminator::kTF:
      return "TF";
    case PathTerminator::kIF:
      return "IF";
    case PathTerminator::kDF:
      return "DF";
    case PathTerminator::kCF:
      return "CF";
    case PathTerminator::kAF:
      return "AF";
    case PathTerminator::kRF:
      return "RF";
    case PathTerminator::kCA:
      return "CA";
    case PathTerminator::kFA:
      return "FA";
    case PathTerminator::kVA:
      return "VA";
    case PathTerminator::kHA:
      return "HA";
    case PathTerminator::kCD:
      return "CD";
    case PathTerminator::kFD:
      return "FD";
    case PathTerminator::kVD:
      return "VD";
    case PathTerminator::kCI:
      return "CI";
    case PathTerminator::kVI:
      return "VI";
    case PathTerminator::kCR:
      return "CR";
    case PathTerminator::kVR:
      return "VR";
    case PathTerminator::kFC:
      return "FC";
    case PathTerminator::kFM:
      return "FM";
    case PathTerminator::kVM:
      return "VM";
    case PathTerminator::kPI:
      return "PI";
    case PathTerminator::kHM:
      return "HM";
    case PathTerminator::kHF:
      return "HF";
    case PathTerminator::kUnknown:
      return "??";
  }
  return "??";
}

bool IsMaptDesc(std::string_view desc) {
  // Right-pad to 4 characters with spaces; character 4 (1-based) == 'M'.
  char pad[4] = {' ', ' ', ' ', ' '};
  const size_t n = desc.size() < 4 ? desc.size() : 4;
  for (size_t i = 0; i < n; ++i) {
    pad[i] = desc[i];
  }
  return pad[3] == 'M';
}

std::string NormalizeRunwayIdent(std::string_view rwy) {
  if (rwy.empty()) {
    return {};
  }
  if (rwy.starts_with(kRunwayPrefix)) {
    return std::string(rwy);
  }
  std::string out(kRunwayPrefix);
  out.append(rwy);
  return out;
}

std::string ApproachRunwayFromName(std::string_view name) {
  // ^[A-Z](\d{1,2})([LRC]?) — skip the ARINC approach-type prefix, take runway
  // digits + optional parallel suffix; trailing variant letters (Y/Z/-M/…) are
  // ignored. Circling names (second character not a digit) return empty.
  if (name.size() < 2 || !std::isupper(static_cast<unsigned char>(name[0]))) {
    return {};
  }
  if (!std::isdigit(static_cast<unsigned char>(name[1]))) {
    return {};
  }
  size_t i = 1;
  while (i < name.size() && std::isdigit(static_cast<unsigned char>(name[i])) && i < 3) {
    ++i;
  }
  // At most two digits (runway 01–36); a third digit is not runway numbering.
  const size_t dig_end = i;
  char suffix = '\0';
  if (dig_end < name.size()) {
    const char c = name[dig_end];
    if (c == 'L' || c == 'R' || c == 'C') {
      suffix = c;
    }
  }
  std::string out(kRunwayPrefix);
  out.append(name.substr(1, dig_end - 1));
  if (suffix != '\0') {
    out.push_back(suffix);
  }
  return out;
}

int ParseRouteTypeToken(std::string_view token) {
  if (token.empty()) {
    return 0;
  }
  int rt = 0;
  const char* begin = token.data();
  const char* end = begin + token.size();
  const auto [ptr, ec] = std::from_chars(begin, end, rt);
  if (ec == std::errc{} && ptr == end) {
    return rt;
  }
  // IAP route types are a single alpha character ('A', 'R', 'I', …).
  if (token.size() == 1) {
    return static_cast<unsigned char>(token[0]);
  }
  return 0;
}

}  // namespace bf
