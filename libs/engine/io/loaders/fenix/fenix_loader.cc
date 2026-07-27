// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/fenix/fenix_loader.h"

#include <sqlite3.h>

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "core/domain/airport.h"
#include "core/domain/airway.h"
#include "core/domain/fixed_ident.h"
#include "core/domain/hold_fix.h"
#include "core/domain/ident.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/domain/navaid_detail.h"
#include "core/domain/procedure.h"
#include "core/domain/waypoint.h"
#include "core/result.h"
#include "io/loaders/sqlite_util.h"
#include "io/nav_data.h"

namespace bf {
namespace {

constexpr std::string_view kLoaderName = "fenix";
constexpr int kFeetPerFlightLevel = 100;
constexpr int kUnknownAltitudeFt = 99999;

// ---- db discovery -------------------------------------------------------

// Search source_dir for a Fenix .db3 database, in priority order, falling
// back to the first *.db3 found.
Result<std::string> FindFenixDb(const std::string& source_dir) {
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::is_directory(source_dir, ec)) {
    return Result<std::string>::Err(
        Error(ErrorCode::kDataMissing, "source directory not found: " + source_dir));
  }
  const char* kPreferred[] = {"fenix_navdata.db3", "navdata.db3", "fenix.db3", "nd.db3"};
  for (const char* name : kPreferred) {
    fs::path p = fs::path(source_dir) / name;
    if (fs::exists(p, ec)) {
      return Result<std::string>::Ok(p.string());
    }
  }
  for (const fs::directory_entry& de : fs::directory_iterator(source_dir, ec)) {
    if (de.is_regular_file() && de.path().extension() == ".db3") {
      return Result<std::string>::Ok(de.path().string());
    }
  }
  return Result<std::string>::Err(
      Error(ErrorCode::kDataMissing, "no Fenix .db3 under " + source_dir));
}

// ---- helpers (NULL-safe column access) ----------------------------------

// ColumnInt returns 0 for NULL; use sqlite3_column_type to distinguish
// a genuine zero from a NULL column.
bool IsColumnNull(sqlite3_stmt* stmt, int col) {
  return sqlite3_column_type(stmt, col) == SQLITE_NULL;
}
int ColumnOptInt(sqlite3_stmt* stmt, int col) {
  return IsColumnNull(stmt, col) ? -1 : ColumnInt(stmt, col);
}
double ColumnOptDouble(sqlite3_stmt* stmt, int col) {
  return IsColumnNull(stmt, col) ? -1.0 : ColumnDouble(stmt, col);
}

// ---- navaid type → WaypointKind ----------------------------------------

// Fenix Navaids.Type maps: 1=VOR, 2=DME, 3=NDB, 4=TACAN, 5=VOR-DME,
// 6=VORTAC, 7=ILS/DME, 8=NDB-DME.
WaypointKind NavaidKindFromType(int type_id) {
  switch (type_id) {
    case 1:   // VOR
    case 4:   // TACAN
    case 5:   // VOR-DME
    case 6:   // VORTAC
      return WaypointKind::kVor;
    case 2:   // DME
      return WaypointKind::kDme;
    case 3:   // NDB
    case 8:   // NDB-DME
      return WaypointKind::kNdb;
    default:
      return WaypointKind::kOther;
  }
}

// ---- cycle extraction ---------------------------------------------------

uint32_t ParseFenixCycle(sqlite3* conn) {
  Result<SqliteStmt> stmt = Prepare(conn, "SELECT val FROM config WHERE key='Cycle'");
  if (!stmt) return 0;
  Result<bool> row = Step(stmt.value().get());
  if (!row || !row.value()) return 0;
  int cycle = 0;
  std::string text = ColumnText(stmt.value().get(), 0);
  auto [ptr, ec] = std::from_chars(text.data(), text.data() + text.size(), cycle);
  return (ec == std::errc{}) ? static_cast<uint32_t>(cycle) : 0;
}

// ---- waypoints ----------------------------------------------------------

Result<void> LoadWaypoints(sqlite3* conn, NavData& data) {
  // Build region lookup: WaypointID → Country (ICAO region code).
  std::unordered_map<int, std::string> region_by_id;
  {
    Result<SqliteStmt> s = Prepare(conn, "SELECT ID, Country FROM WaypointLookup");
    if (!s) return Result<void>::Err(s.error());
    sqlite3_stmt* stmt = s.value().get();
    for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
      region_by_id[ColumnInt(stmt, 0)] = ColumnText(stmt, 1);
    }
  }

  // Build navaid kind lookup: NavaidID → Type.
  std::unordered_map<int, int> navaid_type_of;
  {
    Result<SqliteStmt> s = Prepare(conn, "SELECT ID, Type FROM Navaids");
    if (!s) return Result<void>::Err(s.error());
    sqlite3_stmt* stmt = s.value().get();
    for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
      navaid_type_of[ColumnInt(stmt, 0)] = ColumnInt(stmt, 1);
    }
  }

  // 0=ID 1=Ident 2=Latitude 3=Longtitude 4=NavaidID
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT w.ID, w.Ident, w.Latitude, w.Longtitude, w.NavaidID FROM Waypoints w");
  if (!s) return Result<void>::Err(s.error());

  sqlite3_stmt* stmt = s.value().get();
  int skipped_long = 0;
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
    int id = ColumnInt(stmt, 0);
    std::string ident_str = ColumnText(stmt, 1);
    double lat = ColumnDouble(stmt, 2);
    double lon = ColumnDouble(stmt, 3);
    int nav_id = ColumnOptInt(stmt, 4);

    if (ident_str.size() > 7) {
      std::fprintf(stderr, "fenix: skipping waypoint '%s' (ident too long for FixedIdent, %zu > 7)\n",
                   ident_str.c_str(), ident_str.size());
      ++skipped_long;
      continue;
    }

    std::string region;
    auto rit = region_by_id.find(id);
    if (rit != region_by_id.end()) region = rit->second;

    WaypointKind kind = WaypointKind::kFix;
    if (nav_id > 0) {
      auto nit = navaid_type_of.find(nav_id);
      if (nit != navaid_type_of.end()) kind = NavaidKindFromType(nit->second);
    }

    data.waypoints.push_back(Waypoint{
        Ident{ident_str, region}, Coordinate{lat, lon}, kind});
  }
  if (skipped_long > 0) {
    std::fprintf(stderr, "fenix: %d waypoint(s) skipped (ident too long for FixedIdent)\n",
                 skipped_long);
  }
  return Result<void>::Ok();
}

// ---- navaid details -----------------------------------------------------

Result<void> LoadNavaidDetails(sqlite3* conn, NavData& data) {
  // Navigraph NavaidDetail, persisted for display and lookup-only queries.
  // 0=Ident 1=Type 2=Elevation 3=Freq 4=Range
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT Ident, Type, Elevation, Freq, Range FROM Navaids");
  if (!s) return Result<void>::Err(s.error());

  sqlite3_stmt* stmt = s.value().get();
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
    NavaidDetail detail;
    detail.ident = Ident{ColumnText(stmt, 0), ""};
    detail.kind = NavaidKindFromType(ColumnInt(stmt, 1));
    detail.elev_ft = ColumnInt(stmt, 2);
    detail.freq_raw = ColumnInt(stmt, 3);
    detail.range_nm = ColumnDouble(stmt, 4);
    data.navaid_details.push_back(detail);
  }
  return Result<void>::Ok();
}

// ---- airways ------------------------------------------------------------

Result<void> LoadAirways(sqlite3* conn, NavData& data) {
  // Build airway name lookup.
  std::unordered_map<int, std::string> airway_name;
  {
    Result<SqliteStmt> s = Prepare(conn, "SELECT ID, Ident FROM Airways");
    if (!s) return Result<void>::Err(s.error());
    sqlite3_stmt* stmt = s.value().get();
    for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
      airway_name[ColumnInt(stmt, 0)] = ColumnText(stmt, 1);
    }
  }

  // Build waypoint ident/region lookup.
  std::unordered_map<int, Ident> wp_by_id;
  {
    Result<SqliteStmt> s = Prepare(conn,
        "SELECT w.ID, w.Ident, COALESCE(l.Country,'') "
        "FROM Waypoints w LEFT JOIN WaypointLookup l ON w.ID = l.ID");
    if (!s) return Result<void>::Err(s.error());
    sqlite3_stmt* stmt = s.value().get();
    for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
      wp_by_id[ColumnInt(stmt, 0)] = Ident{ColumnText(stmt, 1), ColumnText(stmt, 2)};
    }
  }

  // 0=AirwayID 1=Level 2=Waypoint1ID 3=Waypoint2ID 4=IsStart 5=IsEnd
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT al.AirwayID, al.Level, al.Waypoint1ID, al.Waypoint2ID, "
      "al.IsStart, al.IsEnd FROM AirwayLegs al");
  if (!s) return Result<void>::Err(s.error());

  sqlite3_stmt* stmt = s.value().get();
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
    int aid = ColumnInt(stmt, 0);
    std::string level = ColumnText(stmt, 1);
    int wp1 = ColumnInt(stmt, 2);
    int wp2 = ColumnInt(stmt, 3);

    auto nm = airway_name.find(aid);
    auto fm = wp_by_id.find(wp1);
    auto to = wp_by_id.find(wp2);
    if (nm == airway_name.end() || fm == wp_by_id.end() || to == wp_by_id.end()) continue;

    AirwayConnection conn;
    conn.from = fm->second;
    conn.to = to->second;
    conn.segment.name = nm->second;
    conn.segment.level = ParseAirwayLevel(level);
    // Fenix schema does not encode per-segment direction restrictions;
    // default to bidirectional as in earth_awy.dat rows without 'F'/'B'.
    conn.segment.direction = AirwayDirection::kBoth;
    data.airways.push_back(conn);
  }
  return Result<void>::Ok();
}

// ---- airports -----------------------------------------------------------

Result<void> LoadAirports(sqlite3* conn, NavData& data) {
  // 0=ICAO 1=Latitude 2=Longtitude 3=Elevation
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT ICAO, Latitude, Longtitude, Elevation FROM Airports");
  if (!s) return Result<void>::Err(s.error());

  sqlite3_stmt* stmt = s.value().get();
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
    std::string icao = ColumnText(stmt, 0);
    // Derive the two-letter ICAO region from the airport identifier.
    std::string region = icao.size() >= 2 ? icao.substr(0, 2) : "";
    data.airports.push_back(Airport{
        icao, region,
        Coordinate{ColumnDouble(stmt, 1), ColumnDouble(stmt, 2)},
        ColumnInt(stmt, 3)});
  }
  return Result<void>::Ok();
}

// ---- holdings -----------------------------------------------------------

Result<void> LoadHoldings(sqlite3* conn, NavData& data) {
  // 0=waypoint_identifier 1=region_code 2=icao_code
  // 3=inbound_holding_course 4=leg_time 5=leg_length 6=turn_direction
  // 7=minimum_altitude 8=maximum_altitude 9=holding_speed
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT waypoint_identifier, region_code, icao_code, "
      "inbound_holding_course, leg_time, leg_length, turn_direction, "
      "minimum_altitude, maximum_altitude, holding_speed FROM Holdings");
  if (!s) return Result<void>::Err(s.error());

  sqlite3_stmt* stmt = s.value().get();
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
    HoldFix h;
    h.fix = Ident{ColumnText(stmt, 0), ColumnText(stmt, 1)};
    h.airport_icao = ColumnText(stmt, 2);
    if (h.airport_icao.empty()) h.airport_icao = "ENRT";
    h.inbound_course = ColumnDouble(stmt, 3);
    h.leg_time_min = ColumnOptDouble(stmt, 4);
    h.leg_dist_nm = ColumnOptDouble(stmt, 5);
    h.turn_dir = (ColumnText(stmt, 6) == "L") ? 'L' : 'R';

    int mn = ColumnOptInt(stmt, 7);
    int mx = ColumnOptInt(stmt, 8);
    int spd = ColumnOptInt(stmt, 9);
    h.min_alt_ft = mn > 0 ? mn : 0;
    h.max_alt_ft = (mx > 0 && mx < kUnknownAltitudeFt) ? mx : 0;
    h.speed_limit_kt = spd > 0 ? spd : 0;
    data.hold_fixes.push_back(h);
  }
  return Result<void>::Ok();
}

// ---- MORA grid ----------------------------------------------------------

Result<void> LoadMoraGrid(sqlite3* conn, NavData& data) {
  // Fenix GridMora: one row per 1° lat × 30 columns (mora01..mora30).
  // Values are hundreds of feet as text (e.g. "010" = 1000 ft).
  // 0=starting_latitude 1=starting_longitude 2..31=mora01..mora30
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT starting_latitude, starting_longitude, "
      "mora01,mora02,mora03,mora04,mora05,mora06,mora07,mora08,mora09,mora10,"
      "mora11,mora12,mora13,mora14,mora15,mora16,mora17,mora18,mora19,mora20,"
      "mora21,mora22,mora23,mora24,mora25,mora26,mora27,mora28,mora29,mora30 "
      "FROM GridMora ORDER BY starting_latitude, starting_longitude");
  if (!s) return Result<void>::Err(s.error());

  sqlite3_stmt* stmt = s.value().get();
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
    int start_lat = ColumnInt(stmt, 0);
    int start_lon = ColumnInt(stmt, 1);

    for (int col = 0; col < 30; ++col) {
      std::string txt = ColumnText(stmt, 2 + col);
      if (txt.empty()) continue;
      int mora_val = 0;
      auto [ptr, ec] = std::from_chars(txt.data(), txt.data() + txt.size(), mora_val);
      if (ec != std::errc{} || mora_val == 0) continue;

      int lat = start_lat;
      int lon = start_lon + col;
      if (lat < 0) lat += 180;
      if (lon < 0) lon += 360;
      if (lat >= 0 && lat < 180 && lon >= 0 && lon < 360) {
        data.mora.SetCell(lat, lon, static_cast<int16_t>(mora_val * 100));
      }
    }
  }
  return Result<void>::Ok();
}

// ============================================================================
// MSA -- NOT AVAILABLE
//
// The Fenix navdata schema does NOT include MSA (Minimum Sector Altitude)
// data. There is no equivalent of the DFD `tbl_msa` table or X-Plane's
// `earth_msa.dat`.
//
// NavData::msa stays empty. NavDatabase::MsaForAirport() will return an
// empty vector for every ICAO. This has no effect on route finding -- MSA
// is purely informational (for terminal-area chart display), and is never
// consulted by A*, Yen, or any constraint.
//
// Callers that need terminal sector altitudes must use a DFD-based loader
// (dfd1 / dfd2) or X-Plane 12.
// ============================================================================

// ---- runways (single scan, indexed by airport ID) -----------------------

Result<std::unordered_map<int, std::vector<Runway>>> LoadAllRunways(sqlite3* conn) {
  std::unordered_map<int, std::vector<Runway>> by_airport;
  // 0=AirportID 1=Ident 2=Latitude 3=Longtitude 4=Elevation
  Result<SqliteStmt> s = Prepare(conn,
      "SELECT AirportID, Ident, Latitude, Longtitude, Elevation FROM Runways "
      "ORDER BY AirportID");
  if (!s) {
    return Result<std::unordered_map<int, std::vector<Runway>>>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&]() {
    by_airport[ColumnInt(stmt, 0)].push_back(Runway{
        "RW" + ColumnText(stmt, 1),
        Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)},
        ColumnInt(stmt, 4)});
  });
  if (!rows) {
    return Result<std::unordered_map<int, std::vector<Runway>>>::Err(rows.error());
  }
  return Result<std::unordered_map<int, std::vector<Runway>>>::Ok(std::move(by_airport));
}

// ---- terminal procedures (single scan, dfd1-style) -----------------------

Result<void> LoadProcTable(sqlite3* conn, std::vector<AirportProcedureData>& out,
                           std::unordered_map<int, std::string>& icao_of) {
  // One full scan of Terminals + TerminalLegs, grouped by airport then terminal.
  // Build the airport ICAO lookup on the fly from the Airports table.
  Result<SqliteStmt> ts = Prepare(conn,
      "SELECT t.ID, t.AirportID, t.Proc, t.Name, t.Rwy "
      "FROM Terminals t ORDER BY t.AirportID, t.ID");
  if (!ts) return Result<void>::Err(ts.error());

  // Pre-build waypoint ident/region lookup for leg fix resolution.
  std::unordered_map<int, std::pair<std::string, std::string>> wp_info;
  {
    Result<SqliteStmt> ws = Prepare(conn,
        "SELECT w.ID, w.Ident, COALESCE(l.Country,'') "
        "FROM Waypoints w LEFT JOIN WaypointLookup l ON w.ID = l.ID");
    if (ws) {
      sqlite3_stmt* stmt = ws.value().get();
      for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt))
        wp_info[ColumnInt(stmt, 0)] = {ColumnText(stmt, 1), ColumnText(stmt, 2)};
    }
  }

  // Pre-index TerminalLegs by (TerminalID, Transition). Fenix procedures
  // interleave runway-specific legs (Transition=RW18L etc.) with common legs
  // (Transition=ALL). We split them into separate Procedures keyed by
  // terminal + transition, matching the DFD/PMDG per-runway structure.
  struct LegGroup {
    std::vector<ProcedureLeg> legs;
    std::string transition;
    std::string runway;
  };
  std::unordered_map<int, std::vector<LegGroup>> leg_groups;
  {
    Result<SqliteStmt> ls = Prepare(conn,
        "SELECT tl.TerminalID, tl.Transition, tl.TrackCode, tl.Course, "
        "tl.Distance, tl.Alt, tl.TurnDir, tl.WptID, ex.SpeedLimit "
        "FROM TerminalLegs tl "
        "LEFT JOIN TerminalLegsEx ex ON tl.ID = ex.ID "
        "ORDER BY tl.TerminalID, tl.ID");
    if (!ls) return Result<void>::Err(ls.error());
    sqlite3_stmt* stmt = ls.value().get();
    std::string prev_trans;
    Result<void> rows = ForEachRow(stmt, [&]() {
      ProcedureLeg leg;
      std::string trans = ColumnText(stmt, 1);
      leg.path_term = ParsePathTerminator(ColumnText(stmt, 2));
      if (leg.path_term == PathTerminator::kUnknown) return;

      // Resolve leg fix ident from WptID.
      int wpt_id = ColumnOptInt(stmt, 7);
      if (wpt_id > 0) {
        auto wit = wp_info.find(wpt_id);
        if (wit != wp_info.end()) {
          std::string ident = wit->second.first;
          std::string region = wit->second.second;
          if (ident.size() <= 7) {  // FixedIdent::kIdentCap
            leg.fix = FixedIdent::FromParts(ident, region);
          }
        }
      }

      leg.course_deg = ColumnDouble(stmt, 3);
      leg.distance_nm = ColumnOptDouble(stmt, 4);
      // Fenix altitude: numeric prefix = feet, trailing letter = constraint.
      std::string alt_text = ColumnText(stmt, 5);
      if (!alt_text.empty()) {
        int alt_ft = 0;
        size_t num_end = alt_text.find_first_not_of("0123456789");
        auto num_part = (num_end == std::string::npos)
                            ? std::string_view(alt_text)
                            : std::string_view(alt_text.data(), num_end);
        auto [ptr, ec] = std::from_chars(num_part.data(), num_part.data() + num_part.size(), alt_ft);
        if (ec == std::errc{}) {
          char desc = (num_end != std::string::npos) ? alt_text[num_end] : '\0';
          AltConstraintKind kind = AltConstraintKind::kAt;
          if (desc == '+' || desc == 'A') kind = AltConstraintKind::kAtOrAbove;
          else if (desc == '-') kind = AltConstraintKind::kAtOrBelow;
          else if (desc == 'B') kind = AltConstraintKind::kBetween;
          leg.alt = AltitudeConstraint{kind, alt_ft, 0};
        }
      }
      std::string td = ColumnText(stmt, 6);
      leg.turn_dir = (td == "L") ? 'L' : (td == "R") ? 'R' : '\0';
      double spd = ColumnOptDouble(stmt, 8);
      leg.speed_limit_kt = static_cast<uint16_t>(spd > 0.0 ? spd : 0.0);

      int tid = ColumnInt(stmt, 0);
      auto& groups = leg_groups[tid];
      if (groups.empty() || groups.back().transition != trans) {
        groups.push_back(LegGroup{{}, trans, trans});
      }
      groups.back().legs.push_back(std::move(leg));
    });
    if (!rows) return Result<void>::Err(rows.error());
  }

  // Pre-index common (ALL) legs per TerminalID for appending to each transition.
  std::unordered_map<int, std::vector<ProcedureLeg>> common_legs;
  for (auto& [tid, groups] : leg_groups) {
    for (auto it = groups.begin(); it != groups.end(); ) {
      if (it->transition == "ALL") {
        common_legs[tid] = std::move(it->legs);
        it = groups.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Iterate terminals, emitting one Procedure per transition.
  sqlite3_stmt* stmt = ts.value().get();
  int cur_aid = -1;
  CifpData cifp;

  auto flush_airport = [&]() {
    if (!cifp.procedures.empty()) {
      auto iit = icao_of.find(cur_aid);
      std::string icao = (iit != icao_of.end()) ? iit->second : "";
      if (!icao.empty()) out.emplace_back(icao, std::move(cifp));
    }
    cifp = CifpData{};
  };

  Result<void> rows = ForEachRow(stmt, [&]() {
    int aid = ColumnInt(stmt, 1);
    int tid = ColumnInt(stmt, 0);
    if (aid != cur_aid) {
      flush_airport();
      cur_aid = aid;
    }

    std::string pt = ColumnText(stmt, 2);
    ProcedureType ptype;
    if (pt == "2") ptype = ProcedureType::kSid;
    else if (pt == "1") ptype = ProcedureType::kStar;
    else ptype = ProcedureType::kApproach;
    std::string name = ColumnText(stmt, 3);
    std::string rwy = ColumnText(stmt, 4);

    auto git = leg_groups.find(tid);
    if (git == leg_groups.end()) return;

    auto cit = common_legs.find(tid);

    // Emit the common-segment procedure first (transition="", no runway).
    // This is the most direct path to the exit fix, matching PMDG structure.
    if (cit != common_legs.end() && !cit->second.empty()) {
      Procedure proc;
      proc.type = ptype;
      proc.name = name;
      proc.transition_ident = "";
      proc.runway = "";
      proc.legs = cit->second;
      cifp.procedures.push_back(std::move(proc));
    }

    // Emit per-runway-transition procedures with common legs appended.
    for (auto& group : git->second) {
      Procedure proc;
      proc.type = ptype;
      proc.name = name;
      proc.transition_ident = group.transition;
      proc.runway = group.runway;
      proc.legs = std::move(group.legs);
      if (cit != common_legs.end())
        proc.legs.insert(proc.legs.end(), cit->second.begin(), cit->second.end());
      cifp.procedures.push_back(std::move(proc));
    }
  });
  if (!rows) return Result<void>::Err(rows.error());
  flush_airport();
  return Result<void>::Ok();
}

// ---- airport ICAO lookup -------------------------------------------------

std::unordered_map<int, std::string> LoadAirportIcaoMap(sqlite3* conn) {
  std::unordered_map<int, std::string> icao_of;
  Result<SqliteStmt> s = Prepare(conn, "SELECT ID, ICAO FROM Airports");
  if (!s) return icao_of;
  sqlite3_stmt* stmt = s.value().get();
  for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt))
    icao_of[ColumnInt(stmt, 0)] = ColumnText(stmt, 1);
  return icao_of;
}

}  // namespace

// ============================================================================
// Public interface
// ============================================================================

Result<NavData> FenixLoader::LoadNavData(const std::string& source_dir) const {
  Result<std::string> db_path = FindFenixDb(source_dir);
  if (!db_path) {
    return Result<NavData>::Err(db_path.error());
  }

  Result<sqlite3*> conn_result = AcquireConn(kLoaderName, db_path.value());
  if (!conn_result) {
    return Result<NavData>::Err(conn_result.error());
  }
  sqlite3* conn = conn_result.value();

  NavData data;
  data.cycle = ParseFenixCycle(conn);

  Result<void> r = LoadWaypoints(conn, data);
  if (!r) return Result<NavData>::Err(r.error());

  r = LoadNavaidDetails(conn, data);
  if (!r) return Result<NavData>::Err(r.error());

  r = LoadAirways(conn, data);
  if (!r) return Result<NavData>::Err(r.error());

  r = LoadAirports(conn, data);
  if (!r) return Result<NavData>::Err(r.error());

  r = LoadHoldings(conn, data);
  if (!r) return Result<NavData>::Err(r.error());

  r = LoadMoraGrid(conn, data);
  if (!r) return Result<NavData>::Err(r.error());

  // NOTE: Fenix navdata has NO MSA table — data.msa is deliberately empty.
  // See the "MSA — NOT AVAILABLE" block above for details.

  return Result<NavData>::Ok(std::move(data));
}

Result<std::vector<AirportProcedureData>> FenixLoader::LoadProcedures(
    const std::string& source_dir) const {
  Result<std::string> db_path = FindFenixDb(source_dir);
  if (!db_path) {
    return Result<std::vector<AirportProcedureData>>::Err(db_path.error());
  }
  Result<sqlite3*> conn_r = AcquireConn(kLoaderName, db_path.value());
  if (!conn_r) {
    return Result<std::vector<AirportProcedureData>>::Err(conn_r.error());
  }
  sqlite3* conn = conn_r.value();

  // Preload airport ICAO lookup.
  auto icao_of = LoadAirportIcaoMap(conn);

  // Scan the Terminals+TerminalLegs tables in a single pass (dfd1-style).
  std::vector<AirportProcedureData> out;
  Result<void> r = LoadProcTable(conn, out, icao_of);
  if (!r) return Result<std::vector<AirportProcedureData>>::Err(r.error());

  // Merge runways by airport ID.
  Result<std::unordered_map<int, std::vector<Runway>>> runways = LoadAllRunways(conn);
  if (!runways) return Result<std::vector<AirportProcedureData>>::Err(runways.error());

  for (auto& [aid, rwys] : runways.value()) {
    if (rwys.empty()) continue;
    auto iit = icao_of.find(aid);
    if (iit == icao_of.end()) continue;
    // Find existing entry or add new.
    auto it = std::find_if(out.begin(), out.end(),
                           [&](const AirportProcedureData& a) { return a.first == iit->second; });
    if (it != out.end()) {
      it->second.runways = std::move(rwys);
    } else {
      CifpData cifp;
      cifp.runways = std::move(rwys);
      out.emplace_back(iit->second, std::move(cifp));
    }
  }

  return Result<std::vector<AirportProcedureData>>::Ok(std::move(out));
}

std::optional<CifpData> FenixLoader::LoadProcedure(const std::string& source_dir,
                                                    const std::string& icao) const {
  Result<std::string> db_path = FindFenixDb(source_dir);
  if (!db_path) return std::nullopt;

  Result<sqlite3*> conn_result = AcquireConn(kLoaderName, db_path.value());
  if (!conn_result) return std::nullopt;
  sqlite3* conn = conn_result.value();

  // Resolve airport ID from ICAO code.
  int airport_id = -1;
  {
    Result<SqliteStmt> s = Prepare(conn, "SELECT ID FROM Airports WHERE ICAO = ?");
    if (!s) return std::nullopt;
    sqlite3_bind_text(s.value().get(), 1, icao.c_str(), -1, SQLITE_TRANSIENT);
    Result<bool> row = Step(s.value().get());
    if (row && row.value()) {
      airport_id = ColumnInt(s.value().get(), 0);
    }
  }
  if (airport_id < 0) return std::nullopt;

  CifpData cifp;
  // Runways for this airport.
  {
    Result<SqliteStmt> rs = Prepare(conn,
        "SELECT Ident, Latitude, Longtitude, Elevation FROM Runways WHERE AirportID = ?");
    if (rs) {
      sqlite3_bind_int(rs.value().get(), 1, airport_id);
      sqlite3_stmt* stmt = rs.value().get();
      for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt))
        cifp.runways.push_back(Runway{"RW" + ColumnText(stmt, 0),
            Coordinate{ColumnDouble(stmt, 1), ColumnDouble(stmt, 2)}, ColumnInt(stmt, 3)});
    }
  }
  // Build waypoint ident/region lookup for leg fix resolution.
  std::unordered_map<int, std::pair<std::string, std::string>> wp_info;
  {
    Result<SqliteStmt> ws = Prepare(conn,
        "SELECT w.ID, w.Ident, COALESCE(l.Country,'') "
        "FROM Waypoints w LEFT JOIN WaypointLookup l ON w.ID = l.ID");
    if (ws) {
      sqlite3_stmt* stmt = ws.value().get();
      for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt))
        wp_info[ColumnInt(stmt, 0)] = {ColumnText(stmt, 1), ColumnText(stmt, 2)};
    }
  }

  // Procedures for this airport (dfd1-style per-airport query).
  {
    Result<SqliteStmt> ts = Prepare(conn,
        "SELECT ID, Proc, Name, Rwy FROM Terminals WHERE AirportID = ?");
    if (!ts) return std::nullopt;
    sqlite3_bind_int(ts.value().get(), 1, airport_id);

    std::unordered_map<int, Procedure> procs;
    sqlite3_stmt* stmt = ts.value().get();
    for (Result<bool> row = Step(stmt); row && row.value(); row = Step(stmt)) {
      int tid = ColumnInt(stmt, 0);
      Procedure proc;
      std::string pt = ColumnText(stmt, 1);
      if (pt == "2") proc.type = ProcedureType::kSid;
      else if (pt == "1") proc.type = ProcedureType::kStar;
      else proc.type = ProcedureType::kApproach;
      proc.name = ColumnText(stmt, 2);
      proc.transition_ident = "ALL";
      proc.runway = ColumnText(stmt, 3);
      procs[tid] = proc;
    }

    // Load legs, including WptID for fix resolution.
    Result<SqliteStmt> ls = Prepare(conn,
        "SELECT tl.TerminalID, tl.TrackCode, tl.Course, tl.Distance, tl.Alt, "
        "tl.TurnDir, tl.WptID, ex.SpeedLimit "
        "FROM TerminalLegs tl LEFT JOIN TerminalLegsEx ex ON tl.ID = ex.ID "
        "ORDER BY tl.TerminalID, tl.ID");
    if (!ls) return std::nullopt;
    sqlite3_stmt* lst = ls.value().get();
    for (Result<bool> row = Step(lst); row && row.value(); row = Step(lst)) {
      int tid = ColumnInt(lst, 0);
      auto it = procs.find(tid);
      if (it == procs.end()) continue;
      ProcedureLeg leg;
      leg.path_term = ParsePathTerminator(ColumnText(lst, 1));
      if (leg.path_term == PathTerminator::kUnknown) continue;

      // Resolve fix ident from WptID (column 5, shifted from TurnDir).
      int wpt_id = ColumnOptInt(lst, 5);
      if (wpt_id > 0) {
        auto wit = wp_info.find(wpt_id);
        if (wit != wp_info.end() && wit->second.first.size() <= 7)
          leg.fix = FixedIdent::FromParts(wit->second.first, wit->second.second);
      }

      leg.course_deg = ColumnDouble(lst, 2);
      leg.distance_nm = ColumnOptDouble(lst, 3);
      std::string alt_text = ColumnText(lst, 4);
      if (!alt_text.empty()) {
        int alt_ft = 0;
        size_t num_end = alt_text.find_first_not_of("0123456789");
        auto num_part = (num_end == std::string::npos) ? std::string_view(alt_text)
                                                        : std::string_view(alt_text.data(), num_end);
        auto [ptr, ec] = std::from_chars(num_part.data(), num_part.data() + num_part.size(), alt_ft);
        if (ec == std::errc{}) {
          char desc = (num_end != std::string::npos) ? alt_text[num_end] : '\0';
          AltConstraintKind kind = AltConstraintKind::kAt;
          if (desc == '+' || desc == 'A') kind = AltConstraintKind::kAtOrAbove;
          else if (desc == '-') kind = AltConstraintKind::kAtOrBelow;
          else if (desc == 'B') kind = AltConstraintKind::kBetween;
          leg.alt = AltitudeConstraint{kind, alt_ft, 0};
        }
      }
      std::string td = ColumnText(lst, 5);
      leg.turn_dir = (td == "L") ? 'L' : (td == "R") ? 'R' : '\0';
      double spd = ColumnOptDouble(lst, 6);
      leg.speed_limit_kt = static_cast<uint16_t>(spd > 0.0 ? spd : 0.0);
      it->second.legs.push_back(leg);
    }
    for (auto& kv : procs)
      if (!kv.second.legs.empty()) cifp.procedures.push_back(std::move(kv.second));
  }
  return cifp;
}

}  // namespace bf
