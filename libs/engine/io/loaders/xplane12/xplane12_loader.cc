// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/xplane12/xplane12_loader.h"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>

#include "core/domain/airway.h"
#include "core/domain/hold_fix.h"
#include "core/domain/ident.h"
#include "core/domain/navaid_detail.h"

namespace bf {

namespace {

// X-Plane data files share a header: a one-char line ("I"/"A"), then a line
// containing "Version", then data rows, terminated by a line "99". This reads
// the file and invokes `parse_row` for each data row (as an istringstream),
// skipping the header. Returns false if the file cannot be opened.
template <class RowFn>
bool ForEachDataRow(const std::string& path, RowFn parse_row) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::string line;
  bool in_data = false;
  while (std::getline(in, line)) {
    // Tolerate CRLF line endings: std::getline only strips '\n', leaving a
    // trailing '\r' that would make the literal "99" terminator check below
    // (and any other exact-match parsing) fail. cifp_parser.cc does the same.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (!in_data) {
      // Header ends at the line containing the "Version" token.
      if (line.find("Version") != std::string::npos) {
        in_data = true;
      }
      continue;
    }
    if (line.empty()) {
      continue;
    }
    if (line == "99") {
      break;
    }
    std::istringstream row(line);
    parse_row(row);
  }
  return true;
}

// Extract the AIRAC cycle from a data file's header. X-Plane header lines read
// like:
//   1200 Version - data cycle 2601, build 20260112, metadata FixXP1200. ...
// Returns the cycle, or 0 if not found. Only the first ~5 lines are scanned (the
// header precedes the "Version" data marker). The X-Plane-only `build` field is
// deliberately ignored -- it is not part of Navigraph's generic metadata.
uint32_t ParseCycle(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return 0;
  }
  auto number_after = [](const std::string& s, const std::string& token) -> uint32_t {
    const size_t pos = s.find(token);
    if (pos == std::string::npos) {
      return 0;
    }
    size_t i = pos + token.size();
    while (i < s.size() && !std::isdigit(static_cast<unsigned char>(s[i]))) {
      ++i;
    }
    uint32_t value = 0;
    while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i]))) {
      value = value * 10 + static_cast<uint32_t>(s[i] - '0');
      ++i;
    }
    return value;
  };
  std::string line;
  for (int n = 0; n < 5 && std::getline(in, line); ++n) {
    if (line.find("cycle") != std::string::npos) {
      return number_after(line, "cycle");
    }
  }
  return 0;
}

// Map an earth_nav.dat row code to a routable waypoint kind, or kOther for row
// codes that are not enroute navaids (ILS components, markers, etc.).
WaypointKind NavKindFromRowCode(int code) {
  switch (code) {
    case 2:
      return WaypointKind::kNdb;
    case 3:
      return WaypointKind::kVor;
    case 12:
      return WaypointKind::kDme;
    case 13:
      return WaypointKind::kDme;  // TACAN, treated as a DME-class point
    default:
      return WaypointKind::kOther;
  }
}

}  // namespace

Result<NavData> XPlane12Loader::LoadNavData(const std::string& data_dir) const {
  NavData data;
  // AIRAC cycle from the fix file header (0 if absent; non-fatal).
  data.cycle = ParseCycle(data_dir + "/earth_fix.dat");
  // Track which (ident, region) points already exist so navaids do not add
  // duplicates (e.g. a co-located VOR and DME share an ident).
  std::unordered_set<Ident> seen;

  // --- earth_fix.dat: lat lon ident terminal region type ... ---
  const bool fixes_ok = ForEachDataRow(data_dir + "/earth_fix.dat", [&](std::istringstream& row) {
    double lat = 0;
    double lon = 0;
    std::string ident;
    std::string terminal;
    std::string region;
    if (!(row >> lat >> lon >> ident >> terminal >> region)) {
      return;
    }
    Ident key(ident, region);
    if (seen.insert(key).second) {
      data.waypoints.push_back(Waypoint{key, Coordinate{lat, lon}, WaypointKind::kFix});
    }
  });
  if (!fixes_ok) {
    return Result<NavData>::Err(Error(ErrorCode::kDataMissing, "cannot open earth_fix.dat"));
  }

  // --- earth_nav.dat: code lat lon elev freq range hdg ident terminal region ... ---
  const bool nav_ok = ForEachDataRow(data_dir + "/earth_nav.dat", [&](std::istringstream& row) {
    int code = 0;
    double lat = 0;
    double lon = 0;
    int elev = 0;
    int freq = 0;
    double range = 0;
    double hdg = 0;
    std::string ident;
    std::string terminal;
    std::string region;
    if (!(row >> code >> lat >> lon >> elev >> freq >> range >> hdg >> ident >> terminal >>
          region)) {
      return;
    }
    WaypointKind kind = NavKindFromRowCode(code);
    if (kind == WaypointKind::kOther) {
      return;  // not an enroute-routable navaid
    }
    Ident key(ident, region);
    if (seen.insert(key).second) {
      data.waypoints.push_back(Waypoint{key, Coordinate{lat, lon}, kind});
      data.navaid_details.push_back(NavaidDetail{key, kind, elev, freq, range, hdg});
    }
  });
  if (!nav_ok) {
    return Result<NavData>::Err(Error(ErrorCode::kDataMissing, "cannot open earth_nav.dat"));
  }

  // --- earth_awy.dat: from freg ftype to treg ttype dir hilo baseFL topFL name ---
  const bool awy_ok = ForEachDataRow(data_dir + "/earth_awy.dat", [&](std::istringstream& row) {
    std::string from;
    std::string freg;
    std::string ftype;
    std::string to;
    std::string treg;
    std::string ttype;
    std::string dir;
    int hilo = 0;
    int base_fl = 0;
    int top_fl = 0;
    std::string name;
    if (!(row >> from >> freg >> ftype >> to >> treg >> ttype >> dir >> hilo >> base_fl >> top_fl >>
          name)) {
      return;
    }
    AirwaySegment seg;
    seg.name = name;
    seg.direction = ParseDirection(dir);
    seg.level = (hilo == 2) ? AirwayLevel::kHigh : AirwayLevel::kLow;
    seg.base_fl = base_fl;
    seg.top_fl = top_fl;
    data.airways.push_back(AirwayConnection{Ident(from, freg), Ident(to, treg), seg});
  });
  if (!awy_ok) {
    return Result<NavData>::Err(Error(ErrorCode::kDataMissing, "cannot open earth_awy.dat"));
  }

  // --- earth_aptmeta.dat: icao region lat lon elev ... ---
  const bool apt_ok = ForEachDataRow(data_dir + "/earth_aptmeta.dat", [&](std::istringstream& row) {
    std::string icao;
    std::string region;
    double lat = 0;
    double lon = 0;
    int elev = 0;
    if (!(row >> icao >> region >> lat >> lon >> elev)) {
      return;
    }
    data.airports.push_back(Airport{icao, region, Coordinate{lat, lon}, elev});
  });
  if (!apt_ok) {
    return Result<NavData>::Err(Error(ErrorCode::kDataMissing, "cannot open earth_aptmeta.dat"));
  }

  // --- earth_mora.dat: lat lon0 then 30 MORA values (hundreds of feet). ---
  // Each row covers 30 one-degree cells starting at lon0. MORA is optional: a
  // missing file simply leaves the grid empty (no altitude floor applied).
  ForEachDataRow(data_dir + "/earth_mora.dat", [&](std::istringstream& row) {
    int lat = 0;
    int lon0 = 0;
    if (!(row >> lat >> lon0)) {
      return;
    }
    for (int i = 0; i < 30; ++i) {
      int value = 0;
      if (!(row >> value)) {
        break;
      }
      if (value > 0) {
        data.mora.SetCell(lat, lon0 + i, static_cast<int16_t>(value));
      }
    }
  });

  // --- earth_msa.dat: rowcode center region airport M (bearing alt radius)... ---
  // Each row is one minimum-sector-altitude record: a center fix, its airport,
  // then a sequence of (bearing, altitude in hundreds of feet, radius) triplets
  // ending at a "000 000 0" sentinel. MSA is optional; a missing file simply
  // leaves no sectors.
  ForEachDataRow(data_dir + "/earth_msa.dat", [&](std::istringstream& row) {
    int row_code = 0;
    std::string center;
    std::string region;
    std::string airport;
    std::string magnetic;  // 'M' or 'T' marker preceding the triplets
    if (!(row >> row_code >> center >> region >> airport >> magnetic)) {
      return;
    }
    MsaSector sector;
    sector.center = Ident(center, region);
    sector.airport_icao = airport;
    int bearing = 0;
    int alt = 0;
    int radius = 0;
    while (row >> bearing >> alt >> radius) {
      if (bearing == 0 && alt == 0 && radius == 0) {
        break;  // sentinel terminates the triplet list
      }
      sector.arcs.push_back(MsaArc{bearing, alt, radius});
    }
    if (!sector.arcs.empty()) {
      data.msa.push_back(std::move(sector));
    }
  });

  // --- earth_hold.dat: ident region airport rowcode inbound_course leg_time
  //     leg_dist turn_dir min_alt max_alt speed ---
  // Optional: a missing file simply leaves hold_fixes empty.
  ForEachDataRow(data_dir + "/earth_hold.dat", [&](std::istringstream& row) {
    std::string ident;
    std::string region;
    std::string airport;
    int row_code = 0;
    double inbound_course = 0;
    double leg_time = 0;
    double leg_dist = 0;
    std::string turn;
    int min_alt = 0;
    int max_alt = 0;
    int speed = 0;
    if (!(row >> ident >> region >> airport >> row_code >> inbound_course >> leg_time >> leg_dist >>
          turn >> min_alt >> max_alt >> speed)) {
      return;
    }
    HoldFix h;
    h.fix = Ident(ident, region);
    h.airport_icao = airport;
    h.inbound_course = inbound_course;
    h.leg_time_min = leg_time;
    h.leg_dist_nm = leg_dist;
    h.turn_dir = (turn == "L") ? 'L' : 'R';
    h.min_alt_ft = min_alt;
    h.max_alt_ft = max_alt;
    h.speed_limit_kt = speed;
    data.hold_fixes.push_back(std::move(h));
  });

  return Result<NavData>::Ok(std::move(data));
}

Result<std::vector<AirportProcedureData>> XPlane12Loader::LoadProcedures(
    const std::string& data_dir) const {
  namespace fs = std::filesystem;
  const fs::path cifp_dir = fs::path(data_dir) / "CIFP";
  std::error_code ec;
  if (!fs::is_directory(cifp_dir, ec)) {
    return Result<std::vector<AirportProcedureData>>::Err(
        Error(ErrorCode::kDataMissing, "no CIFP directory under " + data_dir));
  }

  std::vector<AirportProcedureData> out;
  for (const fs::directory_entry& de : fs::directory_iterator(cifp_dir, ec)) {
    if (!de.is_regular_file() || de.path().extension() != ".dat") {
      continue;
    }
    // The ICAO is the file stem, e.g. "CIFP/KJFK.dat" -> "KJFK".
    std::string icao = de.path().stem().string();
    Result<CifpData> parsed = CifpParser::Parse(de.path().string());
    if (!parsed) {
      continue;  // skip an unreadable file; not fatal for the set as a whole
    }
    out.emplace_back(std::move(icao), std::move(parsed).value());
  }
  return Result<std::vector<AirportProcedureData>>::Ok(std::move(out));
}

std::optional<CifpData> XPlane12Loader::LoadProcedure(const std::string& data_dir,
                                                      const std::string& icao) const {
  Result<CifpData> parsed = CifpParser::Parse(data_dir + "/CIFP/" + icao + ".dat");
  if (!parsed) {
    return std::nullopt;
  }
  return std::move(parsed).value();
}

}  // namespace bf
