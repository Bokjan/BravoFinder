// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/dfd2/dfd2_loader.h"

#include <sqlite3.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "core/domain/airport.h"
#include "core/domain/airway.h"
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

// DFD v2 source loader. Mirrors Dfd1Loader's structure; differences from v1.0
// (table-name prefixes, column renames, swapped distance columns, course_flag,
// MORA quadrant) are isolated here. See the loader plan for the full diff.

constexpr std::string_view kLoaderName = "dfd2";

// Feet per flight level (100 ft per FL); DFD stores altitudes in feet, the
// graph works in flight levels.
constexpr int kFeetPerFlightLevel = 100;
// DFD encodes "no ceiling" as maximum_altitude = 99999; treated as unset.
constexpr int kUnknownAltitudeFt = 99999;

// v2 table names -- prefixes are NOT a fixed pattern (tbl_d_vhfnavaids is a
// single letter), so they are listed explicitly.
constexpr std::string_view kTblHeader = "tbl_hdr_header";
constexpr std::string_view kTblAirports = "tbl_pa_airports";
constexpr std::string_view kTblRunways = "tbl_pg_runways";
constexpr std::string_view kTblEnrouteWp = "tbl_ea_enroute_waypoints";
constexpr std::string_view kTblTerminalWp = "tbl_pc_terminal_waypoints";
constexpr std::string_view kTblVhf = "tbl_d_vhfnavaids";
constexpr std::string_view kTblEnrouteNdb = "tbl_db_enroute_ndbnavaids";
constexpr std::string_view kTblTerminalNdb = "tbl_pn_terminal_ndbnavaids";
constexpr std::string_view kTblAirways = "tbl_er_enroute_airways";
constexpr std::string_view kTblSids = "tbl_pd_sids";
constexpr std::string_view kTblStars = "tbl_pe_stars";
constexpr std::string_view kTblIaps = "tbl_pf_iaps";
constexpr std::string_view kTblHoldings = "tbl_ep_holdings";
constexpr std::string_view kTblMsa = "tbl_ps_airport_msa";
constexpr std::string_view kTblGridMora = "tbl_as_grid_mora";

Result<std::string> FindV2Db(const std::string& source_dir) {
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::is_directory(source_dir, ec)) {
    return Result<std::string>::Err(
        Error(ErrorCode::kDataMissing, "source directory not found: " + source_dir));
  }
  fs::path preferred = fs::path(source_dir) / "db.s3db";
  if (fs::exists(preferred, ec)) {
    return Result<std::string>::Ok(preferred.string());
  }
  for (const fs::directory_entry& de : fs::directory_iterator(source_dir, ec)) {
    if (de.is_regular_file() && de.path().extension() == ".s3db") {
      return Result<std::string>::Ok(de.path().string());
    }
  }
  return Result<std::string>::Err(
      Error(ErrorCode::kDataMissing, "no DFD v2 .s3db under " + source_dir));
}

// Verify the DB is DFD v2 (tbl_hdr_header.dataset == "NG_FWDFD"). A missing
// tbl_hdr_header (Prepare fails or no row) or a different dataset means the user
// likely pointed dfd2 at a v1 DB; give an actionable message.
Result<void> CheckV2Header(sqlite3* conn) {
  const std::string sql =
      std::string("SELECT dataset FROM ") + std::string(kTblHeader) + " LIMIT 1";
  Result<SqliteStmt> stmt = Prepare(conn, sql);
  std::string dataset;
  if (stmt) {
    Result<bool> row = Step(stmt.value().get());
    if (row && row.value()) {
      dataset = ColumnText(stmt.value().get(), 0);
    }
  }
  if (dataset != "NG_FWDFD") {
    return Result<void>::Err(
        Error(ErrorCode::kInvalidArgument,
              "not a DFD v2 database (tbl_hdr_header missing or wrong dataset); "
              "for DFD v1 use --loader dfd1"));
  }
  return Result<void>::Ok();
}

// Map a DFD navaid_class first character to a routable WaypointKind (same as v1).
WaypointKind NavaidKindFromClass(const std::string& cls) {
  if (cls.empty()) {
    return WaypointKind::kOther;
  }
  switch (cls[0]) {
    case 'V':
      return WaypointKind::kVor;
    case 'D':
    case 'T':
    case 'M':
      return WaypointKind::kDme;
    default:
      return WaypointKind::kOther;  // I/N/P or space-padded ILS component
  }
}

// Parse a v2 MORA starting coordinate: VarChar like "N07"/"S90"/"W090"/"E179"
// -> signed integer degrees.
int ParseHemiCoord(const std::string& s) {
  if (s.size() < 2) {
    return 0;
  }
  const char hemi = s[0];
  // Only N/S/E/W are valid hemispheres; anything else is malformed. Guarding it
  // stops an unexpected leading byte from being treated as a positive N/E value.
  if (hemi != 'N' && hemi != 'S' && hemi != 'E' && hemi != 'W') {
    return 0;
  }
  int value = 0;
  const auto [ptr, ec] = std::from_chars(s.data() + 1, s.data() + s.size(), value);
  if (ec != std::errc{}) {
    return 0;  // non-numeric magnitude: let the caller skip this MORA row
  }
  if (hemi == 'S' || hemi == 'W') {
    return -value;
  }
  return value;  // N or E
}

// --- Per-table loaders (column names that differ from v1 are noted). ---

Result<void> LoadEnrouteWaypoints(sqlite3* conn, NavData& data, std::unordered_set<Ident>& seen) {
  const std::string sql = std::string(
                              "SELECT icao_code, waypoint_identifier, "
                              "waypoint_latitude, waypoint_longitude FROM ") +
                          std::string(kTblEnrouteWp);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    if (seen.insert(key).second) {
      data.waypoints.push_back(Waypoint{
          key, Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)}, WaypointKind::kFix});
    }
  });
}

Result<void> LoadTerminalWaypoints(sqlite3* conn, NavData& data, std::unordered_set<Ident>& seen) {
  // Use icao_code (2-char ICAO region), NOT region_code (the airport the fix
  // belongs to, e.g. "01OH"): region_code is not a region, mismatches how
  // airways/procedures reference the fix, and overflows FixedIdent::kRegionCap.
  // (Same as dfd1; see the dfd1 LoadTerminalWaypoints comment.)
  const std::string sql = std::string(
                              "SELECT icao_code, waypoint_identifier, "
                              "waypoint_latitude, waypoint_longitude FROM ") +
                          std::string(kTblTerminalWp);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    if (seen.insert(key).second) {
      data.waypoints.push_back(Waypoint{
          key, Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)}, WaypointKind::kFix});
    }
  });
}

Result<void> LoadVhfNavaids(sqlite3* conn, NavData& data, std::unordered_set<Ident>& seen) {
  // v2: navaid_identifier / navaid_latitude / navaid_longitude / navaid_frequency
  // (v1 used vor_*).
  const std::string sql =
      std::string(
          "SELECT icao_code, navaid_identifier, navaid_frequency, navaid_class, "
          "range, station_declination, dme_elevation, navaid_latitude, "
          "navaid_longitude FROM ") +
      std::string(kTblVhf);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const WaypointKind kind = NavaidKindFromClass(ColumnText(stmt, 3));
    if (kind == WaypointKind::kOther) {
      return;  // ILS/LOC component: not enroute-routable
    }
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    if (!seen.insert(key).second) {
      return;
    }
    data.waypoints.push_back(
        Waypoint{key, Coordinate{ColumnDouble(stmt, 7), ColumnDouble(stmt, 8)}, kind});
    const int freq_raw = static_cast<int>(std::lround(ColumnDouble(stmt, 2) * 100.0));
    data.navaid_details.push_back(NavaidDetail{key, kind, ColumnInt(stmt, 6), freq_raw,
                                               ColumnDouble(stmt, 4), ColumnDouble(stmt, 5)});
  });
}

Result<void> LoadNdbNavaids(sqlite3* conn, NavData& data, std::string_view table,
                            std::unordered_set<Ident>& seen) {
  // v2: navaid_identifier / navaid_frequency / navaid_latitude / navaid_longitude
  // (v1 used ndb_*).
  const std::string sql = std::string(
                              "SELECT icao_code, navaid_identifier, navaid_frequency, range, "
                              "navaid_latitude, navaid_longitude FROM ") +
                          std::string(table);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    if (!seen.insert(key).second) {
      return;
    }
    data.waypoints.push_back(Waypoint{key, Coordinate{ColumnDouble(stmt, 4), ColumnDouble(stmt, 5)},
                                      WaypointKind::kNdb});
    data.navaid_details.push_back(
        NavaidDetail{key, WaypointKind::kNdb, 0, ColumnInt(stmt, 2), ColumnDouble(stmt, 3), 0.0});
  });
}

Result<void> LoadAirways(sqlite3* conn, NavData& data) {
  // Column names are the same as v1; only the table name differs.
  const std::string sql =
      std::string(
          "SELECT route_identifier, seqno, waypoint_identifier, icao_code, "
          "direction_restriction, flightlevel, minimum_altitude1, maximum_altitude, "
          "outbound_course, inbound_course, waypoint_description_code FROM ") +
      std::string(kTblAirways) + " ORDER BY route_identifier, seqno";
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  bool have_prev = false;
  // route_identifier is NOT unique per physical airway; description-code column 2
  // == 'E' ("End of Airway") flags the last fix of each same-name string, so the
  // prev->current chain must break there or disjoint strings get a phantom leg.
  // (Same as dfd1; see the dfd1 LoadAirways comment for the full rationale.)
  bool prev_is_awy_end = false;
  std::string prev_route, prev_ident, prev_icao, prev_dir, prev_level;
  int prev_min_alt = 0, prev_max_alt = 0;
  return ForEachRow(stmt, [&]() {
    const std::string route = ColumnText(stmt, 0);
    const std::string ident = ColumnText(stmt, 2);
    const std::string icao = ColumnText(stmt, 3);
    if (have_prev && route == prev_route && !prev_is_awy_end) {
      // Each airway record's direction/level/altitude describe the OUTBOUND leg
      // leaving that fix (in increasing-seqno order), so the segment prev->current
      // takes its attributes from the PREVIOUS row, not the current one. Reading
      // them off the current row shifts every restriction one leg forward -- e.g.
      // a 'F' meant for the JMU->IJ leg would wrongly bar the IGADO->JMU leg,
      // breaking otherwise-valid routes (regression: "G212 does not connect JMU").
      // (Same as dfd1.)
      AirwaySegment seg;
      seg.name = prev_route;
      seg.direction = ParseDirection(prev_dir);
      seg.level = ParseAirwayLevel(prev_level);
      seg.base_fl = prev_min_alt / kFeetPerFlightLevel;  // feet -> flight level
      // 99999 ("no ceiling") -> 999: a very high finite band, not the
      // base_fl==0 && top_fl==0 sentinel AltitudeBandConstraint exempts. Harmless
      // since no query cruises above FL999. (Same as dfd1.)
      seg.top_fl = prev_max_alt / kFeetPerFlightLevel;
      data.airways.push_back(
          AirwayConnection{Ident(prev_ident, prev_icao), Ident(ident, icao), seg});
    }
    prev_route = route;
    prev_ident = ident;
    prev_icao = icao;
    prev_dir = ColumnText(stmt, 4);
    prev_level = ColumnText(stmt, 5);
    prev_min_alt = ColumnInt(stmt, 6);
    prev_max_alt = ColumnInt(stmt, 7);
    // Column 2 ('E') of the waypoint description code marks End of Airway. Read
    // raw (untrimmed): trimming a blank column 1 would shift the byte offsets and
    // silently miss the 'E', reintroducing cross-instance phantom legs. (Same as dfd1.)
    const std::string desc = ColumnTextRaw(stmt, 10);
    prev_is_awy_end = desc.size() > 1 && desc[1] == 'E';
    have_prev = true;
  });
}

Result<void> LoadAirports(sqlite3* conn, NavData& data) {
  const std::string sql = std::string(
                              "SELECT icao_code, airport_identifier, "
                              "airport_ref_latitude, airport_ref_longitude, elevation FROM ") +
                          std::string(kTblAirports);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    data.airports.push_back(Airport{ColumnText(stmt, 1), ColumnText(stmt, 0),
                                    Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)},
                                    ColumnInt(stmt, 4)});
  });
}

// Airport magnetic variation by ICAO, for converting course_flag='T' (true
// course) legs to magnetic. Read by the procedure loaders only.
Result<std::unordered_map<std::string, double>> LoadAirportMagvars(sqlite3* conn) {
  std::unordered_map<std::string, double> out;
  const std::string sql = std::string("SELECT airport_identifier, magnetic_variation FROM ") +
                          std::string(kTblAirports);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<std::unordered_map<std::string, double>>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&]() { out[ColumnText(stmt, 0)] = ColumnDouble(stmt, 1); });
  if (!rows) {
    return Result<std::unordered_map<std::string, double>>::Err(rows.error());
  }
  return Result<std::unordered_map<std::string, double>>::Ok(std::move(out));
}

Result<void> LoadHoldings(sqlite3* conn, NavData& data) {
  const std::string sql =
      std::string(
          "SELECT region_code, icao_code, waypoint_identifier, inbound_holding_course, "
          "turn_direction, leg_length, leg_time, minimum_altitude, maximum_altitude, "
          "holding_speed FROM ") +
      std::string(kTblHoldings);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    HoldFix h;
    h.fix = Ident(ColumnText(stmt, 2), ColumnText(stmt, 1));
    h.airport_icao = ColumnText(stmt, 0);
    h.inbound_course = ColumnDouble(stmt, 3);
    const std::string turn = ColumnText(stmt, 4);
    // Hold turn direction defaults to right ('R') when absent/unknown, matching
    // the X-Plane earth_hold parse (see dfd1_loader for the same convention).
    h.turn_dir = (turn == "L") ? 'L' : 'R';
    h.leg_dist_nm = ColumnDouble(stmt, 5);
    h.leg_time_min = ColumnDouble(stmt, 6);
    h.min_alt_ft = ColumnInt(stmt, 7);
    const int max_alt = ColumnInt(stmt, 8);
    h.max_alt_ft = (max_alt >= kUnknownAltitudeFt) ? 0 : max_alt;
    h.speed_limit_kt = ColumnInt(stmt, 9);
    data.hold_fixes.push_back(std::move(h));
  });
}

Result<void> LoadMsa(sqlite3* conn, NavData& data) {
  const std::string sql =
      std::string(
          "SELECT airport_identifier, msa_center, radius_limit, "
          "sector_bearing_1, sector_altitude_1, sector_bearing_2, sector_altitude_2, "
          "sector_bearing_3, sector_altitude_3, sector_bearing_4, sector_altitude_4, "
          "sector_bearing_5, sector_altitude_5 FROM ") +
      std::string(kTblMsa);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    MsaSector sector;
    sector.center = Ident{ColumnText(stmt, 1), {}};
    sector.airport_icao = ColumnText(stmt, 0);
    const int radius = ColumnInt(stmt, 2);
    for (int i = 0; i < 5; ++i) {
      const int bearing = ColumnInt(stmt, 3 + i * 2);
      const int alt = ColumnInt(stmt, 4 + i * 2);
      if (bearing == 0 && alt == 0) {
        break;
      }
      sector.arcs.push_back(MsaArc{bearing, alt, radius});
    }
    if (!sector.arcs.empty()) {
      data.msa.push_back(std::move(sector));
    }
  });
}

Result<void> LoadGridMora(sqlite3* conn, NavData& data) {
  // v2: quadrant_code subdivides each 1-degree cell into 4; the blank-quadrant row
  // already carries the full 1-degree value (verified), so we take only those and
  // skip A/B/C/D (the 1-degree MoraGrid cannot store the 30-min offset). Starting
  // coords are VarChar "N07"/"W090", unlike v1's INTEGER.
  const std::string sql =
      std::string(
          "SELECT starting_latitude, starting_longitude, quadrant_code, "
          "mora01, mora02, mora03, mora04, mora05, mora06, mora07, mora08, mora09, mora10, "
          "mora11, mora12, mora13, mora14, mora15, mora16, mora17, mora18, mora19, mora20, "
          "mora21, mora22, mora23, mora24, mora25, mora26, mora27, mora28, mora29, mora30 "
          "FROM ") +
      std::string(kTblGridMora);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const std::string q = ColumnText(stmt, 2);
    if (!q.empty()) {
      return;  // A/B/C/D quadrant: skip (blank row covers this cell at 1 degree)
    }
    const int lat = ParseHemiCoord(ColumnText(stmt, 0));
    const int lon0 = ParseHemiCoord(ColumnText(stmt, 1));
    for (int i = 0; i < 30; ++i) {
      const std::string v = ColumnText(stmt, 3 + i);
      if (v.empty() || v == "UNK") {
        continue;  // inner loop: skip this cell, not the whole row
      }
      int value = 0;
      const auto [ptr, ec] = std::from_chars(v.data(), v.data() + v.size(), value);
      if (ec != std::errc{}) {
        continue;  // non-numeric cell (not empty / not "UNK"): skip, keep no MORA
      }
      if (value > 0) {
        data.mora.SetCell(lat, lon0 + i, static_cast<int16_t>(value));
      }
    }
  });
}

// v2 procedure column layout (differs from v1: course+course_flag, swapped
// distance value/flag columns).
struct ProcCols {
  int airport = 0;       // airport_identifier
  int proc = 1;          // procedure_identifier
  int route_type = 2;    // route_type
  int transition = 3;    // transition_identifier
  int seqno = 4;         // seqno
  int wp_ident = 5;      // waypoint_identifier
  int wp_icao = 6;       // waypoint_icao_code
  int path_term = 7;     // path_termination
  int course = 8;        // course (REAL)
  int course_flag = 9;   // course_flag ('M'/'T')
  int dist_value = 10;   // distance_time (REAL value -- swapped vs v1!)
  int dist_flag = 11;    // route_distance_holding_distance_time (flag -- swapped!)
  int alt_desc = 12;     // altitude_description
  int alt1 = 13;         // altitude1
  int alt2 = 14;         // altitude2
  int rnp = 15;          // rnp (REAL, nautical miles)
  int turn_dir = 16;     // turn_direction ('L'/'R')
  int speed_limit = 17;  // speed_limit (knots)
};

// SQL for one procedure table, in ProcCols column order.
std::string ProcSql(std::string_view table, bool single_airport) {
  std::string sql = std::string(
                        "SELECT airport_identifier, procedure_identifier, route_type, "
                        "transition_identifier, seqno, waypoint_identifier, waypoint_icao_code, "
                        "path_termination, course, course_flag, distance_time, "
                        "route_distance_holding_distance_time, altitude_description, altitude1, "
                        "altitude2, rnp, turn_direction, speed_limit FROM ") +
                    std::string(table);
  if (single_airport) {
    sql += " WHERE airport_identifier = ?";
  }
  // route_type is part of the flush key, so it must be in the ORDER BY to give
  // the partitioning a total order: without it, rows sharing (name, transition)
  // but differing in route_type could interleave and split one procedure across
  // two records (the SQL engine is free to return same-key rows in any order).
  sql +=
      " ORDER BY airport_identifier, procedure_identifier, transition_identifier, "
      "route_type, seqno";
  return sql;
}

// Emit one Procedure leg from a row. `magvar` converts true courses (course_flag
// 'T'); magnetic courses ('M'/blank) are used as-is.
void AppendLeg(sqlite3_stmt* stmt, const ProcCols& c, double magvar, Procedure& proc) {
  ProcedureLeg leg;
  leg.fix = FixedIdent::FromParts(ColumnText(stmt, c.wp_ident), ColumnText(stmt, c.wp_icao));
  leg.path_term = ParsePathTerminator(ColumnText(stmt, c.path_term));
  const double course = ColumnDouble(stmt, c.course);
  const std::string flag = ColumnText(stmt, c.course_flag);
  leg.course_deg = (flag == "T") ? ToMagnetic(course, magvar) : course;
  // v2 distance: distance_time holds the value, route_distance_... holds the flag.
  const std::string dflag = ColumnText(stmt, c.dist_flag);
  if (dflag == "D") {
    leg.distance_nm = ColumnDouble(stmt, c.dist_value);
  }
  leg.alt = ParseAltConstraint(ColumnText(stmt, c.alt_desc), ColumnInt(stmt, c.alt1),
                               ColumnInt(stmt, c.alt2));
  const double rnp = ColumnDouble(stmt, c.rnp);
  leg.rnp_centinm = rnp > 0.0 ? static_cast<uint16_t>(std::lround(rnp * 100.0)) : 0;
  const std::string turn = ColumnText(stmt, c.turn_dir);
  leg.turn_dir = (turn == "L") ? 'L' : (turn == "R") ? 'R' : '\0';
  leg.speed_limit_kt = static_cast<uint16_t>(ColumnInt(stmt, c.speed_limit));
  proc.legs.push_back(std::move(leg));
}

// Full-table scan appending per-airport procedures into `out`. magvar_by_airport
// supplies the true->magnetic conversion for course_flag='T' legs.
Result<void> LoadProcTable(sqlite3* conn, std::string_view table, ProcedureType type,
                           const std::unordered_map<std::string, double>& magvar_by_airport,
                           std::vector<AirportProcedureData>& out) {
  Result<SqliteStmt> s = Prepare(conn, ProcSql(table, /*single_airport=*/false));
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  const ProcCols c;

  Procedure current;
  current.type = type;
  bool have_current = false;
  std::string cur_airport, cur_name, cur_trans, cur_route;
  CifpData cifp;
  double magvar = 0.0;

  auto flush_proc = [&]() {
    if (have_current && !current.legs.empty()) {
      cifp.procedures.push_back(std::move(current));
    }
    current = Procedure{};
    current.type = type;
    have_current = false;
  };
  auto flush_airport = [&]() {
    flush_proc();
    if (!cifp.procedures.empty() || !cifp.runways.empty()) {
      out.emplace_back(cur_airport, std::move(cifp));
    }
    cifp = CifpData{};
  };

  Result<void> rows = ForEachRow(stmt, [&]() {
    const std::string airport = ColumnText(stmt, c.airport);
    const std::string name = ColumnText(stmt, c.proc);
    const std::string trans = ColumnText(stmt, c.transition);
    const std::string route = ColumnText(stmt, c.route_type);
    if (airport != cur_airport) {
      flush_airport();
      cur_airport = airport;
      if (auto it = magvar_by_airport.find(airport); it != magvar_by_airport.end()) {
        magvar = it->second;
      } else {
        magvar = 0.0;
      }
    } else if (have_current && (name != cur_name || trans != cur_trans || route != cur_route)) {
      flush_proc();
    }
    if (!have_current) {
      cur_name = name;
      cur_trans = trans;
      cur_route = route;
      current.name = name;
      current.transition_ident = trans;
      if (trans.rfind("RW", 0) == 0) {
        current.runway = trans;
      }
      int rt = 0;
      std::from_chars(route.data(), route.data() + route.size(), rt);
      current.route_type = rt;
      have_current = true;
    }
    AppendLeg(stmt, c, magvar, current);
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  // Flush the last airport's accumulated procedures.
  flush_airport();
  return Result<void>::Ok();
}

Result<std::unordered_map<std::string, std::vector<Runway>>> LoadAllRunways(sqlite3* conn) {
  std::unordered_map<std::string, std::vector<Runway>> by_airport;
  const std::string sql = std::string(
                              "SELECT airport_identifier, runway_identifier, runway_latitude, "
                              "runway_longitude, landing_threshold_elevation FROM ") +
                          std::string(kTblRunways) + " ORDER BY airport_identifier";
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<std::unordered_map<std::string, std::vector<Runway>>>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&]() {
    Runway rwy;
    rwy.ident = ColumnText(stmt, 1);
    rwy.threshold = Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)};
    rwy.elevation_ft = ColumnInt(stmt, 4);
    by_airport[ColumnText(stmt, 0)].push_back(std::move(rwy));
  });
  if (!rows) {
    return Result<std::unordered_map<std::string, std::vector<Runway>>>::Err(rows.error());
  }
  return Result<std::unordered_map<std::string, std::vector<Runway>>>::Ok(std::move(by_airport));
}

void MergeRunways(std::vector<AirportProcedureData>& out,
                  std::unordered_map<std::string, std::vector<Runway>>& runways) {
  for (AirportProcedureData& ap : out) {
    auto it = runways.find(ap.first);
    if (it != runways.end()) {
      ap.second.runways = std::move(it->second);
      runways.erase(it);
    }
  }
  for (auto& [airport, rwys] : runways) {
    if (!rwys.empty()) {
      CifpData cifp;
      cifp.runways = std::move(rwys);
      out.emplace_back(airport, std::move(cifp));
    }
  }
}

// Load one airport on demand (LoadProcedure path). Returns nullopt when the
// airport has no procedures/runways; a SQLite step error degrades to nullopt
// too (this path is best-effort -- the full LoadProcedures path propagates such
// errors).
std::optional<CifpData> LoadAirportProcedures(
    sqlite3* conn, const std::string& airport,
    const std::unordered_map<std::string, double>& magvar_by_airport) {
  double magvar = 0.0;
  if (auto it = magvar_by_airport.find(airport); it != magvar_by_airport.end()) {
    magvar = it->second;
  }
  CifpData cifp;
  const std::pair<std::string_view, ProcedureType> kTables[] = {
      {kTblSids, ProcedureType::kSid},
      {kTblStars, ProcedureType::kStar},
      {kTblIaps, ProcedureType::kApproach},
  };
  for (const auto& [table, type] : kTables) {
    Result<SqliteStmt> s = Prepare(conn, ProcSql(table, /*single_airport=*/true));
    if (!s) {
      continue;
    }
    sqlite3_stmt* stmt = s.value().get();
    sqlite3_bind_text(stmt, 1, airport.c_str(), -1, SQLITE_TRANSIENT);
    const ProcCols c;
    Procedure current;
    current.type = type;
    bool have_current = false;
    std::string cur_name, cur_trans, cur_route;
    auto flush_current = [&]() {
      if (have_current && !current.legs.empty()) {
        cifp.procedures.push_back(std::move(current));
      }
      current = Procedure{};
      current.type = type;
      have_current = false;
    };
    Result<void> rows = ForEachRow(stmt, [&]() {
      const std::string name = ColumnText(stmt, c.proc);
      const std::string trans = ColumnText(stmt, c.transition);
      const std::string route = ColumnText(stmt, c.route_type);
      if (have_current && (name != cur_name || trans != cur_trans || route != cur_route)) {
        flush_current();
      }
      if (!have_current) {
        cur_name = name;
        cur_trans = trans;
        cur_route = route;
        current.name = name;
        current.transition_ident = trans;
        if (trans.rfind("RW", 0) == 0) {
          current.runway = trans;
        }
        int rt = 0;
        std::from_chars(route.data(), route.data() + route.size(), rt);
        current.route_type = rt;
        have_current = true;
      }
      AppendLeg(stmt, c, magvar, current);
    });
    if (!rows) {
      return std::nullopt;  // step error: treat as load failure for this airport
    }
    flush_current();
  }
  Result<SqliteStmt> rs =
      Prepare(conn, std::string("SELECT runway_identifier, runway_latitude, runway_longitude, "
                                "landing_threshold_elevation FROM ") +
                        std::string(kTblRunways) + " WHERE airport_identifier = ?");
  if (rs) {
    sqlite3_stmt* stmt = rs.value().get();
    sqlite3_bind_text(stmt, 1, airport.c_str(), -1, SQLITE_TRANSIENT);
    Result<void> rows = ForEachRow(stmt, [&]() {
      Runway rwy;
      rwy.ident = ColumnText(stmt, 0);
      rwy.threshold = Coordinate{ColumnDouble(stmt, 1), ColumnDouble(stmt, 2)};
      rwy.elevation_ft = ColumnInt(stmt, 3);
      cifp.runways.push_back(std::move(rwy));
    });
    if (!rows) {
      return std::nullopt;
    }
  }
  if (cifp.procedures.empty() && cifp.runways.empty()) {
    return std::nullopt;
  }
  return cifp;
}

}  // namespace

// Convert a true course to magnetic using the airport's magnetic variation.
// magnetic = true - variation (variation: west negative, so magnetic > true west).
// Defined at namespace scope (declared in the header) so the sign convention can
// be locked by a unit test.
// Degrees in a full circle; used to wrap magnetic variation into [0, 360).
constexpr double kDegreesFullCircle = 360.0;

double ToMagnetic(double true_course, double magvar) {
  double mag = true_course - magvar;
  while (mag < 0.0) {
    mag += kDegreesFullCircle;
  }
  while (mag >= kDegreesFullCircle) {
    mag -= kDegreesFullCircle;
  }
  return mag;
}

Result<NavData> Dfd2Loader::LoadNavData(const std::string& source_dir) const {
  Result<std::string> db_path = FindV2Db(source_dir);
  if (!db_path) {
    return Result<NavData>::Err(std::move(db_path).error());
  }
  Result<sqlite3*> conn = AcquireConn(kLoaderName, db_path.value());
  if (!conn) {
    return Result<NavData>::Err(std::move(conn).error());
  }
  Result<void> header = CheckV2Header(conn.value());
  if (!header) {
    return Result<NavData>::Err(std::move(header).error());
  }

  NavData data;
  // AIRAC cycle from tbl_hdr_header.cycle (e.g. "2601").
  const std::string cycle_sql =
      std::string("SELECT cycle FROM ") + std::string(kTblHeader) + " LIMIT 1";
  Result<SqliteStmt> s = Prepare(conn.value(), cycle_sql);
  if (s) {
    Result<bool> row = Step(s.value().get());
    if (!row) {
      return Result<NavData>::Err(row.error());
    }
    if (row.value()) {
      const std::string cyc = ColumnText(s.value().get(), 0);
      unsigned int cycle = 0;
      std::from_chars(cyc.data(), cyc.data() + cyc.size(), cycle);
      data.cycle = cycle;
    }
  }

  // Each per-table loader returns Result<void>; a step error (corruption, IO)
  // propagates as a load failure rather than silently producing partial data.
  std::unordered_set<Ident> seen;
  Result<void> load = Result<void>::Ok();
  load = LoadEnrouteWaypoints(conn.value(), data, seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadVhfNavaids(conn.value(), data, seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadNdbNavaids(conn.value(), data, kTblEnrouteNdb, seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadNdbNavaids(conn.value(), data, kTblTerminalNdb, seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  // Terminal waypoints load LAST so canonical enroute/navaid entries win the
  // first-wins `seen` dedup once terminal fixes are keyed by icao_code. (Same as dfd1.)
  load = LoadTerminalWaypoints(conn.value(), data, seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadAirways(conn.value(), data);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadAirports(conn.value(), data);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadHoldings(conn.value(), data);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadMsa(conn.value(), data);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadGridMora(conn.value(), data);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  return Result<NavData>::Ok(std::move(data));
}

Result<std::vector<AirportProcedureData>> Dfd2Loader::LoadProcedures(
    const std::string& source_dir) const {
  Result<std::string> db_path = FindV2Db(source_dir);
  if (!db_path) {
    return Result<std::vector<AirportProcedureData>>::Err(std::move(db_path).error());
  }
  Result<sqlite3*> conn = AcquireConn(kLoaderName, db_path.value());
  if (!conn) {
    return Result<std::vector<AirportProcedureData>>::Err(std::move(conn).error());
  }
  // Defensive: LoadProcedures is only reached after NavDatabase::Open ->
  // LoadNavData already validated the header, but validate here too so a future
  // caller that invokes the loader directly (bypassing Open) gets the actionable
  // "for DFD v1 use --loader dfd1" error instead of an obscure kParseError.
  if (Result<void> header = CheckV2Header(conn.value()); !header) {
    return Result<std::vector<AirportProcedureData>>::Err(std::move(header).error());
  }
  // magvar is needed for course_flag='T' legs; read just the (airport, magvar)
  // pairs rather than re-running the full airports load.
  Result<std::unordered_map<std::string, double>> magvar = LoadAirportMagvars(conn.value());
  if (!magvar) {
    return Result<std::vector<AirportProcedureData>>::Err(magvar.error());
  }

  std::vector<AirportProcedureData> out;
  Result<void> load = Result<void>::Ok();
  load = LoadProcTable(conn.value(), kTblSids, ProcedureType::kSid, magvar.value(), out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  load = LoadProcTable(conn.value(), kTblStars, ProcedureType::kStar, magvar.value(), out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  load = LoadProcTable(conn.value(), kTblIaps, ProcedureType::kApproach, magvar.value(), out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  std::sort(out.begin(), out.end(),
            [](const AirportProcedureData& a, const AirportProcedureData& b) {
              return a.first < b.first;
            });
  std::vector<AirportProcedureData> merged;
  for (AirportProcedureData& ap : out) {
    if (!merged.empty() && merged.back().first == ap.first) {
      auto& dst = merged.back().second.procedures;
      auto& src = ap.second.procedures;
      dst.insert(dst.end(), std::make_move_iterator(src.begin()),
                 std::make_move_iterator(src.end()));
    } else {
      merged.push_back(std::move(ap));
    }
  }
  Result<std::unordered_map<std::string, std::vector<Runway>>> runways =
      LoadAllRunways(conn.value());
  if (!runways) {
    return Result<std::vector<AirportProcedureData>>::Err(runways.error());
  }
  MergeRunways(merged, runways.value());
  return Result<std::vector<AirportProcedureData>>::Ok(std::move(merged));
}

std::optional<CifpData> Dfd2Loader::LoadProcedure(const std::string& source_dir,
                                                  const std::string& icao) const {
  Result<std::string> db_path = FindV2Db(source_dir);
  if (!db_path) {
    return std::nullopt;
  }
  Result<sqlite3*> conn = AcquireConn(kLoaderName, db_path.value());
  if (!conn) {
    return std::nullopt;
  }
  // On-demand: read just this airport's magvar (one row) rather than the whole
  // airports table.
  std::unordered_map<std::string, double> magvar;
  Result<SqliteStmt> ms =
      Prepare(conn.value(), std::string("SELECT airport_identifier, magnetic_variation FROM ") +
                                std::string(kTblAirports) + " WHERE airport_identifier = ?");
  if (ms) {
    sqlite3_stmt* stmt = ms.value().get();
    sqlite3_bind_text(stmt, 1, icao.c_str(), -1, SQLITE_TRANSIENT);
    Result<bool> row = Step(stmt);
    if (!row) {
      return std::nullopt;  // step error: treat as load failure for this airport
    }
    if (row.value()) {
      magvar[ColumnText(stmt, 0)] = ColumnDouble(stmt, 1);
    }
  }
  return LoadAirportProcedures(conn.value(), icao, magvar);
}

}  // namespace bf
