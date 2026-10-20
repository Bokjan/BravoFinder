// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/xplane12/xplane12_loader.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include "core/base/log.h"
#include "core/domain/airway.h"
#include "core/domain/arinc_codes.h"
#include "core/domain/fixed_string.h"
#include "core/domain/hold_fix.h"
#include "core/domain/ident.h"
#include "core/domain/navaid_detail.h"
#include "io/loaders/point_dedup.h"
#include "io/loaders/xplane12/cifp_parser.h"

namespace bf {

namespace {

// X-Plane data files share a header: a one-char line ("I"/"A"), then a line
// containing "Version", then data rows, terminated by a line "99". This reads
// the file and invokes `parse_row` for each data row (as an istringstream),
// skipping the header. Returns false if the file cannot be opened. A row the
// parser could not fully read (the stream is left failed, e.g. a truncated or
// non-numeric field) is counted and reported once per file instead of passing
// silently.
template <class RowFn>
bool ForEachDataRow(const std::string& path, RowFn parse_row) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return false;
  }
  std::string line;
  bool in_data = false;
  int malformed = 0;
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
    if (row.fail()) {
      ++malformed;
    }
  }
  if (malformed > 0) {
    BF_LOG_WARN("xplane12: {} malformed row(s) in {}", malformed, path);
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

// The class of point an ARINC reference names: a fix (earth_fix.dat), a VHF
// navaid (VOR/DME/TACAN), or an NDB (both earth_nav.dat).
enum class PointClass : uint8_t { kFix, kVhf, kNdb };

PointClass ClassOfKind(WaypointKind kind) {
  switch (kind) {
    case WaypointKind::kNdb:
      return PointClass::kNdb;
    case WaypointKind::kVor:
    case WaypointKind::kDme:
      return PointClass::kVhf;
    default:
      return PointClass::kFix;
  }
}

// Published positions of every earth_fix / earth_nav point, keyed by what a
// reference to it can name: (ident, region, class, owner). `owner` is the
// row's terminal-area column -- "ENRT" or the owning airport's ICAO -- for fixes
// and NDBs (terminal NDBs are published per airport too); VHF navaids are keyed
// with an empty owner since references to them never name an airport. Used to
// give airway endpoints and CIFP legs the position hint that tells same-key
// points apart. First row wins on a duplicate key.
class PointIndex {
 public:
  void Add(const std::string& ident, const std::string& region, PointClass cls,
           std::string_view owner, const Coordinate& coord) {
    by_key_.try_emplace(Key(ident, region, cls, cls == PointClass::kVhf ? "" : owner), coord);
  }

  std::optional<Coordinate> Find(std::string_view ident, std::string_view region, PointClass cls,
                                 std::string_view owner) const {
    auto it = by_key_.find(Key(ident, region, cls, cls == PointClass::kVhf ? "" : owner));
    if (it == by_key_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  // Locate a CIFP leg's fix for airport `icao` from its ARINC 424 section and
  // subsection: E/A enroute waypoint, P/C the airport's terminal waypoint, D/' '
  // VHF navaid, D/B enroute NDB, P/N the airport's terminal NDB. Anything else
  // (runway P/G, localizer P/I, ...) is not a graph vertex and gets no hint.
  std::optional<Coordinate> Locate(std::string_view ident, std::string_view region, char section,
                                   char subsection, std::string_view icao) const {
    if (section == 'E' && subsection == 'A') {
      return Find(ident, region, PointClass::kFix, kEnrouteOwner);
    }
    if (section == 'P' && subsection == 'C') {
      return Find(ident, region, PointClass::kFix, icao);
    }
    if (section == 'D' && subsection == ' ') {
      return Find(ident, region, PointClass::kVhf, "");
    }
    if (section == 'D' && subsection == 'B') {
      return Find(ident, region, PointClass::kNdb, kEnrouteOwner);
    }
    if (section == 'P' && subsection == 'N') {
      return Find(ident, region, PointClass::kNdb, icao);
    }
    return std::nullopt;
  }

  static constexpr std::string_view kEnrouteOwner = "ENRT";

 private:
  static std::string Key(std::string_view ident, std::string_view region, PointClass cls,
                         std::string_view owner) {
    std::string key;
    key.reserve(ident.size() + region.size() + owner.size() + 4);
    key.append(ident).push_back('\0');
    key.append(region).push_back('\0');
    key.push_back(static_cast<char>('0' + static_cast<int>(cls)));
    key.append(owner);
    return key;
  }

  std::unordered_map<std::string, Coordinate> by_key_;
};

// One earth_fix.dat row: lat lon ident terminal region ...
struct FixRow {
  double lat = 0;
  double lon = 0;
  std::string ident;
  std::string terminal;
  std::string icao_code;
};
bool ReadFixRow(std::istringstream& row, FixRow& out) {
  return static_cast<bool>(row >> out.lat >> out.lon >> out.ident >> out.terminal >> out.icao_code);
}

// One earth_nav.dat row: code lat lon elev freq range hdg ident terminal region ...
struct NavRow {
  int code = 0;
  double lat = 0;
  double lon = 0;
  int elev = 0;
  int freq = 0;
  double range = 0;
  double hdg = 0;
  std::string ident;
  std::string terminal;
  std::string icao_code;
};
bool ReadNavRow(std::istringstream& row, NavRow& out) {
  return static_cast<bool>(row >> out.code >> out.lat >> out.lon >> out.elev >> out.freq >>
                           out.range >> out.hdg >> out.ident >> out.terminal >> out.icao_code);
}

// Build the PointIndex from earth_fix.dat + earth_nav.dat alone (the CIFP load
// paths, which do not otherwise parse the enroute files). A missing file just
// leaves its points unindexed: legs then carry no hint.
PointIndex LoadPointIndex(const std::string& data_dir) {
  PointIndex index;
  ForEachDataRow(data_dir + "/earth_fix.dat", [&](std::istringstream& row) {
    FixRow r;
    if (ReadFixRow(row, r)) {
      index.Add(r.ident, r.icao_code, PointClass::kFix, r.terminal, Coordinate{r.lat, r.lon});
    }
  });
  ForEachDataRow(data_dir + "/earth_nav.dat", [&](std::istringstream& row) {
    NavRow r;
    if (!ReadNavRow(row, r)) {
      return;
    }
    const WaypointKind kind = NavKindFromRowCode(r.code);
    if (kind != WaypointKind::kOther) {
      index.Add(r.ident, r.icao_code, ClassOfKind(kind), r.terminal, Coordinate{r.lat, r.lon});
    }
  });
  return index;
}

// A FixLocator for airport `icao` over `index` (which must outlive it).
FixLocator LocatorFor(const PointIndex& index, const std::string& icao) {
  return [&index, icao](std::string_view ident, std::string_view region, char section,
                        char subsection) {
    return index.Locate(ident, region, section, subsection, icao);
  };
}

// The PointClass an earth_awy.dat endpoint type names: 11 = fix, 2 = NDB,
// 3 = VHF navaid.
std::optional<PointClass> AirwayEndpointClass(const std::string& type) {
  if (type == "11") {
    return PointClass::kFix;
  }
  if (type == "2") {
    return PointClass::kNdb;
  }
  if (type == "3") {
    return PointClass::kVhf;
  }
  return std::nullopt;
}

}  // namespace

Result<NavData> XPlane12Loader::LoadNavData(const std::string& data_dir) const {
  NavData data;
  // AIRAC cycle from the fix file header (0 if absent; non-fatal).
  data.cycle = ParseCycle(data_dir + "/earth_fix.dat");
  // Merge rows that are the same physical point (e.g. a co-located VOR and DME
  // share an ident) while keeping distinct same-key points such as per-airport
  // terminal fixes (earth_fix's terminal column names the owning airport).
  PointDeduper seen("xplane12");
  // Positions by what an airway endpoint names, for the airway position hints.
  PointIndex index;

  // --- earth_fix.dat: lat lon ident terminal region type ... ---
  const bool fixes_ok = ForEachDataRow(data_dir + "/earth_fix.dat", [&](std::istringstream& row) {
    FixRow r;
    if (!ReadFixRow(row, r)) {
      return;
    }
    const Coordinate coord{r.lat, r.lon};
    index.Add(r.ident, r.icao_code, PointClass::kFix, r.terminal, coord);
    Ident key(r.ident, r.icao_code);
    if (seen.Insert(key, coord)) {
      data.waypoints.push_back(Waypoint{std::move(key), coord, WaypointKind::kFix});
    }
  });
  if (!fixes_ok) {
    return Result<NavData>::Err(Error(ErrorCode::kDataMissing, "cannot open earth_fix.dat"));
  }

  // --- earth_nav.dat: code lat lon elev freq range hdg ident terminal region ... ---
  const bool nav_ok = ForEachDataRow(data_dir + "/earth_nav.dat", [&](std::istringstream& row) {
    NavRow r;
    if (!ReadNavRow(row, r)) {
      return;
    }
    WaypointKind kind = NavKindFromRowCode(r.code);
    if (kind == WaypointKind::kOther) {
      return;  // not an enroute-routable navaid
    }
    const Coordinate coord{r.lat, r.lon};
    index.Add(r.ident, r.icao_code, ClassOfKind(kind), r.terminal, coord);
    Ident key(r.ident, r.icao_code);
    if (seen.Insert(key, coord)) {
      data.waypoints.push_back(Waypoint{key, coord, kind});
      data.navaid_details.push_back(NavaidDetail{key, kind, r.elev, r.freq, r.range, r.hdg});
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
    // Endpoint type 11/2/3 says which table the point is in; airway fixes are
    // enroute, so look them up under the ENRT owner.
    auto hint = [&](const std::string& ident, const std::string& region,
                    const std::string& type) -> std::optional<Coordinate> {
      const std::optional<PointClass> cls = AirwayEndpointClass(type);
      return cls ? index.Find(ident, region, *cls, PointIndex::kEnrouteOwner) : std::nullopt;
    };
    data.airways.push_back(AirwayConnection{Ident(from, freg), Ident(to, treg), seg,
                                            hint(from, freg, ftype), hint(to, treg, ttype)});
  });
  if (!awy_ok) {
    return Result<NavData>::Err(Error(ErrorCode::kDataMissing, "cannot open earth_awy.dat"));
  }

  // --- earth_aptmeta.dat: icao region lat lon elev ... ---
  const bool apt_ok = ForEachDataRow(data_dir + "/earth_aptmeta.dat", [&](std::istringstream& row) {
    std::string icao;
    std::string icao_code;
    double lat = 0;
    double lon = 0;
    int elev = 0;
    if (!(row >> icao >> icao_code >> lat >> lon >> elev)) {
      return;
    }
    data.airports.push_back(Airport{icao, icao_code, Coordinate{lat, lon}, elev});
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
    std::string icao_code;
    std::string airport;
    std::string magnetic;  // 'M' or 'T' marker preceding the triplets
    if (!(row >> row_code >> center >> icao_code >> airport >> magnetic)) {
      return;
    }
    MsaSector sector;
    sector.center = Ident(center, icao_code);
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
    std::string icao_code;
    std::string airport;
    int row_code = 0;
    double inbound_course = 0;
    double leg_time = 0;
    double leg_dist = 0;
    std::string turn;
    int min_alt = 0;
    int max_alt = 0;
    int speed = 0;
    if (!(row >> ident >> icao_code >> airport >> row_code >> inbound_course >> leg_time >>
          leg_dist >> turn >> min_alt >> max_alt >> speed)) {
      return;
    }
    HoldFix h;
    h.fix = Ident(ident, icao_code);
    h.airport_icao = airport;
    h.inbound_course = inbound_course;
    h.leg_time_min = leg_time;
    h.leg_dist_nm = leg_dist;
    h.turn_dir = EqualsCode(turn, kTurnLeft) ? kTurnLeft : kTurnRight;
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

  // CIFP legs carry no positions; look each fix up in the enroute files so the
  // legs can be wired to the right one of several same-key points.
  const PointIndex index = LoadPointIndex(data_dir);
  std::vector<AirportProcedureData> out;
  for (const fs::directory_entry& de : fs::directory_iterator(cifp_dir, ec)) {
    if (!de.is_regular_file() || de.path().extension() != ".dat") {
      continue;
    }
    // The ICAO is the file stem, e.g. "CIFP/KJFK.dat" -> "KJFK". A stem that
    // cannot be an airport key (a stray file such as "KJFK.backup.dat") is
    // skipped: the cache could not store it.
    std::string icao = de.path().stem().string();
    if (!FixedName8::From(icao)) {
      BF_LOG_WARN("xplane12: skipping CIFP file '{}' (not an airport ICAO)", de.path().string());
      continue;
    }
    Result<CifpData> parsed = CifpParser::Parse(de.path().string(), LocatorFor(index, icao));
    if (!parsed) {
      continue;  // skip an unreadable file; not fatal for the set as a whole
    }
    out.emplace_back(std::move(icao), std::move(parsed).value());
  }
  // directory_iterator order is filesystem-defined; sort so the output (and the
  // .bfdb built from it) is reproducible across machines and runs.
  std::sort(out.begin(), out.end(),
            [](const AirportProcedureData& a, const AirportProcedureData& b) {
              return a.first < b.first;
            });
  return Result<std::vector<AirportProcedureData>>::Ok(std::move(out));
}

Result<std::optional<CifpData>> XPlane12Loader::LoadProcedure(const std::string& data_dir,
                                                              const std::string& icao) const {
  // An airport without a CIFP file simply has no procedures; a file that is
  // there but cannot be parsed is a read failure, not an empty airport.
  const std::string path = data_dir + "/CIFP/" + icao + ".dat";
  std::error_code ec;
  if (!std::filesystem::is_regular_file(path, ec)) {
    return Result<std::optional<CifpData>>::Ok(std::nullopt);
  }
  const PointIndex index = LoadPointIndex(data_dir);
  Result<CifpData> parsed = CifpParser::Parse(path, LocatorFor(index, icao));
  if (!parsed) {
    return Result<std::optional<CifpData>>::Err(std::move(parsed).error());
  }
  return Result<std::optional<CifpData>>::Ok(std::move(parsed).value());
}

}  // namespace bf
