// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/xplane12/cifp_parser.h"

#include <array>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

namespace bf {

namespace {

// Trim leading/trailing ASCII spaces from a view into `s`.
std::string Trim(std::string_view s) {
  size_t b = 0;
  size_t e = s.size();
  while (b < e && s[b] == ' ') {
    ++b;
  }
  while (e > b && s[e - 1] == ' ') {
    --e;
  }
  return std::string(s.substr(b, e - b));
}

// Split `s` on `delim` into trimmed-as-stored fields (no trimming here; callers
// trim what they need so column positions stay exact).
std::vector<std::string> Split(std::string_view s, char delim) {
  std::vector<std::string> out;
  size_t start = 0;
  while (true) {
    const size_t pos = s.find(delim, start);
    if (pos == std::string_view::npos) {
      out.emplace_back(s.substr(start));
      break;
    }
    out.emplace_back(s.substr(start, pos - start));
    start = pos + 1;
  }
  return out;
}

// Field index (0-based, after splitting a leg line on ',') for each datum.
// Verified against cycle 2601 CIFP files.
constexpr int kRouteType = 1;
constexpr int kProcName = 2;
constexpr int kTransition = 3;
constexpr int kFixIdent = 4;
constexpr int kFixRegion = 5;
constexpr int kTurnDir = 9;  // turn direction: 'L'/'R' (or blank)
constexpr int kRnp = 10;     // required navigation performance, ARINC-encoded
constexpr int kPathTerm = 11;
constexpr int kCourse = 20;      // magnetic course, tenths of a degree
constexpr int kDistance = 21;    // leg distance, tenths of a nautical mile
constexpr int kAltDesc = 22;     // altitude descriptor: + - @ B (or blank)
constexpr int kAlt1 = 23;        // altitude one, feet
constexpr int kAlt2 = 24;        // altitude two, feet (lower bound for 'B')
constexpr int kSpeedLimit = 27;  // speed limit, knots (0/blank if none)
// Row-length gate for a leg record. Intentionally 25 (not kSpeedLimit+1=28):
// speed_limit is the only field past index 24, and FieldInt() bounds-checks its
// index, so a 25-27 field row is accepted and reads speed_limit as 0 rather than
// being dropped for lacking a trailing (optional) speed-limit column.
constexpr int kMinLegFields = 25;

// Read a field as an integer, treating blank/non-numeric as 0.
int FieldInt(const std::vector<std::string>& f, int idx) {
  if (idx < 0 || idx >= static_cast<int>(f.size())) {
    return 0;
  }
  const std::string t = Trim(f[idx]);
  if (t.empty()) {
    return 0;
  }
  return std::atoi(t.c_str());
}

std::string FieldStr(const std::vector<std::string>& f, int idx) {
  if (idx < 0 || idx >= static_cast<int>(f.size())) {
    return {};
  }
  return Trim(f[idx]);
}

// Decode the CIFP RNP field into hundredths of a nautical mile. The field is the
// 3-character ARINC 424 form: the first two digits are the mantissa and the
// third is a negative power of ten, so "302" = 30 x 10^-2 = 0.30 NM (-> 30) and
// "010" = 01 x 10^-0 = 1.0 NM (-> 100). Blank/non-numeric fields yield 0.
uint16_t RnpCentinm(const std::string& field) {
  if (field.size() != 3) {
    return 0;  // absent or not the expected 3-character encoding
  }
  for (const char c : field) {
    if (c < '0' || c > '9') {
      return 0;
    }
  }
  const int mantissa = (field[0] - '0') * 10 + (field[1] - '0');
  const int exp = field[2] - '0';
  // centinm = mantissa x 10^-exp x 100 = mantissa x 10^(2-exp).
  const int power = 2 - exp;
  long long centinm = mantissa;
  if (power >= 0) {
    for (int i = 0; i < power; ++i) {
      centinm *= 10;
    }
  } else {
    // Fractional hundredths (exp > 2): round to the nearest centinm.
    long long div = 1;
    for (int i = 0; i < -power; ++i) {
      div *= 10;
    }
    centinm = (centinm + div / 2) / div;
  }
  if (centinm < 0 || centinm > 65535) {
    return 0;
  }
  return static_cast<uint16_t>(centinm);
}

// Normalize a turn-direction field to 'L'/'R', or '\0' when unspecified.
char TurnDir(const std::string& field) {
  if (field == "L") {
    return 'L';
  }
  if (field == "R") {
    return 'R';
  }
  return '\0';
}

ProcedureType TypeFromTag(std::string_view tag) {
  if (tag == "STAR") {
    return ProcedureType::kStar;
  }
  if (tag == "APPCH") {
    return ProcedureType::kApproach;
  }
  return ProcedureType::kSid;
}

// Convert an X-Plane packed coordinate (e.g. "N40372318" = 40 deg 37 min
// 23.18 sec, "W073470505" = 073 deg 47 min 05.05 sec) to signed degrees.
double PackedToDegrees(std::string_view s) {
  if (s.size() < 2) {
    return 0.0;
  }
  const char hemi = s[0];
  const bool is_lon = (hemi == 'E' || hemi == 'W');
  const bool negative = (hemi == 'S' || hemi == 'W');
  const std::string digits(s.substr(1));
  const size_t deg_len = is_lon ? 3 : 2;
  if (digits.size() < deg_len + 2) {
    return 0.0;
  }
  const double deg = std::atoi(digits.substr(0, deg_len).c_str());
  const double min = std::atoi(digits.substr(deg_len, 2).c_str());
  // Remaining digits are seconds with two implied decimal places (SSss).
  const std::string rest = digits.substr(deg_len + 2);
  const double sec = rest.empty() ? 0.0 : std::atof(rest.c_str()) / 100.0;
  const double value = deg + min / 60.0 + sec / 3600.0;
  return negative ? -value : value;
}

// Parse one RWY record into a Runway, or return false if it lacks coordinates.
// Format: `RWY:RW04L,...,<elev>,...;<lat>,<lon>,<tch>;`
bool ParseRunway(std::string_view line, Runway& out) {
  const std::vector<std::string> parts = Split(line, ';');
  if (parts.size() < 2) {
    return false;
  }
  const std::vector<std::string> head = Split(parts[0], ',');
  if (head.empty()) {
    return false;
  }
  // head[0] is "RWY:RW04L"; the runway ident follows the colon.
  const std::vector<std::string> tag = Split(head[0], ':');
  if (tag.size() < 2) {
    return false;
  }
  out.ident = Trim(tag[1]);
  out.elevation_ft = FieldInt(head, 3);  // threshold elevation, feet
  const std::vector<std::string> coords = Split(parts[1], ',');
  if (coords.size() < 2) {
    return false;
  }
  out.threshold = Coordinate{PackedToDegrees(Trim(coords[0])), PackedToDegrees(Trim(coords[1]))};
  return true;
}

}  // namespace

CifpData CifpParser::ParseLines(const std::vector<std::string>& lines) {
  CifpData data;

  // Group consecutive legs sharing (type, name, transition, route type) into
  // one Procedure. A change in any of these starts a new procedure.
  ProcedureType cur_type = ProcedureType::kSid;
  std::string cur_name;
  std::string cur_trans;
  int cur_route = -1;
  bool have_current = false;

  auto flush = [&](Procedure& p) {
    if (!p.legs.empty()) {
      data.procedures.push_back(std::move(p));
    }
  };
  Procedure current;

  for (const std::string& raw : lines) {
    if (raw.empty()) {
      continue;
    }
    const size_t colon = raw.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string_view tag(raw.data(), colon);

    if (tag == "RWY") {
      Runway rwy;
      if (ParseRunway(raw, rwy)) {
        data.runways.push_back(std::move(rwy));
      }
      continue;
    }
    if (tag != "SID" && tag != "STAR" && tag != "APPCH") {
      continue;  // PRDAT and anything else
    }

    // Drop the trailing record terminator and everything after it.
    std::string_view body(raw);
    const size_t semi = body.find(';');
    if (semi != std::string_view::npos) {
      body = body.substr(0, semi);
    }
    const std::vector<std::string> f = Split(body, ',');
    if (static_cast<int>(f.size()) < kMinLegFields) {
      continue;  // malformed / too short to carry a path terminator + altitudes
    }

    const ProcedureType type = TypeFromTag(tag);
    const int route_type = FieldInt(f, kRouteType);
    const std::string name = FieldStr(f, kProcName);
    const std::string trans = FieldStr(f, kTransition);

    if (!have_current || type != cur_type || name != cur_name || trans != cur_trans ||
        route_type != cur_route) {
      flush(current);
      current = Procedure{};
      current.type = type;
      current.name = name;
      current.transition_ident = trans;
      current.route_type = route_type;
      // A runway transition's identifier is the runway name prefixed with "RW"
      // (e.g. "RW04L"); record it as the procedure's runway filter key.
      if (trans.rfind("RW", 0) == 0) {
        current.runway = trans;
      }
      cur_type = type;
      cur_name = name;
      cur_trans = trans;
      cur_route = route_type;
      have_current = true;
    }

    ProcedureLeg leg;
    leg.fix = FixedIdent::FromParts(FieldStr(f, kFixIdent), FieldStr(f, kFixRegion));
    leg.path_term = ParsePathTerminator(FieldStr(f, kPathTerm));
    leg.course_deg = FieldInt(f, kCourse) / 10.0;
    leg.distance_nm = FieldInt(f, kDistance) / 10.0;
    leg.alt = ParseAltConstraint(FieldStr(f, kAltDesc), FieldInt(f, kAlt1), FieldInt(f, kAlt2));
    leg.rnp_centinm = RnpCentinm(FieldStr(f, kRnp));
    leg.speed_limit_kt = static_cast<uint16_t>(FieldInt(f, kSpeedLimit));
    leg.turn_dir = TurnDir(FieldStr(f, kTurnDir));
    current.legs.push_back(std::move(leg));
  }
  flush(current);

  return data;
}

Result<CifpData> CifpParser::Parse(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return Result<CifpData>::Err(Error(ErrorCode::kDataMissing, "cannot open CIFP file: " + path));
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();  // tolerate CRLF
    }
    lines.push_back(std::move(line));
  }
  return Result<CifpData>::Ok(ParseLines(lines));
}

}  // namespace bf
