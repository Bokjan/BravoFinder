// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/dfd2/dfd2_loader.h"

#include <sqlite3.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "core/base/log.h"
#include "core/domain/airport.h"
#include "core/domain/airway.h"
#include "core/domain/arinc_codes.h"
#include "core/domain/coordinate.h"
#include "core/domain/encoding_scale.h"
#include "core/domain/fixed_string.h"
#include "core/domain/hold_fix.h"
#include "core/domain/ident.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/domain/nav_tokens.h"
#include "core/domain/navaid_detail.h"
#include "core/domain/procedure.h"
#include "core/domain/waypoint.h"
#include "core/result.h"
#include "io/fs_util.h"
#include "io/loaders/loader_constants.h"
#include "io/loaders/nav_data.h"
#include "io/loaders/point_dedup.h"
#include "io/loaders/sqlite_util.h"

namespace bf {
namespace {

// DFD v2 source loader. Mirrors Dfd1Loader's structure; differences from v1.0
// (table-name prefixes, column renames, swapped distance columns, course_flag,
// MORA quadrant) are isolated here. See the loader plan for the full diff.

constexpr std::string_view kLoaderName = "dfd2";

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
  Result<std::vector<std::string>> found = ListFiles(source_dir, ".s3db");
  if (!found) {
    return Result<std::string>::Err(std::move(found).error());
  }
  if (!found.value().empty()) {
    return Result<std::string>::Ok(std::move(found.value().front()));
  }
  return Result<std::string>::Err(
      Error(ErrorCode::kDataMissing, "no DFD v2 .s3db under " + source_dir));
}

// Verify the DB is DFD v2 (tbl_hdr_header.dataset == "NG_FWDFD"). A missing
// tbl_hdr_header (Prepare fails or no row) or a different dataset means the user
// likely pointed dfd2 at a v1 DB; give an actionable message.
Result<void> CheckV2Header(sqlite3* conn) {
  const std::string sql = std::format("SELECT dataset FROM {} LIMIT 1", kTblHeader);
  Result<SqliteStmt> stmt = Prepare(conn, sql);
  std::string dataset;
  if (stmt) {
    Result<bool> row = Step(stmt.value().get());
    if (!row) {
      // The table exists but cannot be read (I/O error, corruption): report that,
      // not "wrong database".
      return Result<void>::Err(row.error());
    }
    if (row.value()) {
      dataset = ColumnText(stmt.value().get(), 0);
    }
  }
  if (dataset != "NG_FWDFD") {
    return Result<void>::Err(Error(
        ErrorCode::kInvalidArgument,
        std::format("not a DFD v2 database ({} missing or wrong dataset); for DFD v1 use --loader "
                    "dfd1",
                    kTblHeader)));
  }
  return Result<void>::Ok();
}

// Map a DFD navaid_class to a routable WaypointKind (same as v1; see the class
// layout there).
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
      return WaypointKind::kOther;  // I/N/P: ILS or MLS DME component
  }
}

// Parse a v2 MORA starting coordinate: VarChar like "N07"/"S90"/"W090"/"E179"
// -> signed integer degrees, or nullopt when malformed (the caller skips the
// row rather than filing its 30 cells under 0 degrees).
std::optional<int> ParseHemiCoord(const std::string& s) {
  if (s.size() < 2) {
    return std::nullopt;
  }
  const char hemi = s[0];
  // Only N/S/E/W are valid hemispheres; anything else is malformed. Guarding it
  // stops an unexpected leading byte from being treated as a positive N/E value.
  if (hemi != 'N' && hemi != 'S' && hemi != 'E' && hemi != 'W') {
    return std::nullopt;
  }
  int value = 0;
  const char* end = s.data() + s.size();
  const auto [ptr, ec] = std::from_chars(s.data() + 1, end, value);
  if (ec != std::errc{} || ptr != end) {
    return std::nullopt;  // non-numeric or trailing junk ("N07X")
  }
  if (hemi == 'S' || hemi == 'W') {
    return -value;
  }
  return value;  // N or E
}

// --- Per-table loaders (column names that differ from v1 are noted). ---

Result<void> LoadEnrouteWaypoints(sqlite3* conn, NavData& data, PointDeduper& seen) {
  const std::string sql = std::format(
      "SELECT icao_code, waypoint_identifier, waypoint_latitude, waypoint_longitude FROM {}",
      kTblEnrouteWp);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    const std::optional<Coordinate> coord = ColumnCoord(stmt, 2, 3);
    if (!coord) {
      BF_LOG_WARN("dfd2: skipping '{}/{}' (no position)", key.ident, key.arinc424_icao_code);
      return;
    }
    if (seen.Insert(key, *coord)) {
      data.waypoints.push_back(Waypoint{key, *coord, WaypointKind::kFix});
    }
  });
}

Result<void> LoadTerminalWaypoints(sqlite3* conn, NavData& data, PointDeduper& seen) {
  // Use icao_code (2-char ICAO region), NOT region_code (the airport the fix
  // belongs to, e.g. "01OH"): region_code is not a region, mismatches how
  // airways/procedures reference the fix, and overflows FixedIdent::kArinc424IcaoCodeCap.
  // (Same as dfd1; see the dfd1 LoadTerminalWaypoints comment.)
  const std::string sql = std::format(
      "SELECT icao_code, waypoint_identifier, waypoint_latitude, waypoint_longitude FROM {}",
      kTblTerminalWp);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    const std::optional<Coordinate> coord = ColumnCoord(stmt, 2, 3);
    if (!coord) {
      BF_LOG_WARN("dfd2: skipping '{}/{}' (no position)", key.ident, key.arinc424_icao_code);
      return;
    }
    if (seen.Insert(key, *coord)) {
      data.waypoints.push_back(Waypoint{key, *coord, WaypointKind::kFix});
    }
  });
}

Result<void> LoadVhfNavaids(sqlite3* conn, NavData& data, PointDeduper& seen) {
  // v2: navaid_identifier / navaid_latitude / navaid_longitude / navaid_frequency
  // (v1 used vor_*). A DME-only station (class " D", TACAN "T", ...) has no VOR
  // and leaves navaid_latitude/longitude NULL; its position is the DME's (2601:
  // 4689 rows, all with dme_latitude). Reading only navaid_* put 1238 routable
  // DME/TACANs at (0, 0) and gave airway T645 a 6697 NM NANOO->UQQ leg.
  const std::string sql = std::format(
      "SELECT icao_code, navaid_identifier, navaid_frequency, navaid_class, range, "
      "station_declination, dme_elevation, COALESCE(navaid_latitude, dme_latitude), "
      "COALESCE(navaid_longitude, dme_longitude) FROM {}",
      kTblVhf);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const WaypointKind kind = NavaidKindFromClass(ColumnText(stmt, 3));
    if (kind == WaypointKind::kOther) {
      // ILS/MLS DME component: not enroute-routable. The X-Plane and Fenix
      // loaders keep these as vertices (kDme / kOther); none carries an
      // airway edge, so the difference only shows in ident lookups.
      return;
    }
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    const std::optional<Coordinate> coord = ColumnCoord(stmt, 7, 8);
    if (!coord) {
      BF_LOG_WARN("dfd2: skipping '{}/{}' (no position)", key.ident, key.arinc424_icao_code);
      return;
    }
    if (!seen.Insert(key, *coord)) {
      return;
    }
    data.waypoints.push_back(Waypoint{key, *coord, kind});
    const int freq_raw = static_cast<int>(std::lround(ColumnDouble(stmt, 2) * kCentiScale));
    data.navaid_details.push_back(NavaidDetail{key, kind, ColumnInt(stmt, 6), freq_raw,
                                               ColumnDouble(stmt, 4), ColumnDouble(stmt, 5)});
  });
}

Result<void> LoadNdbNavaids(sqlite3* conn, NavData& data, std::string_view table,
                            PointDeduper& seen) {
  // v2: navaid_identifier / navaid_frequency / navaid_latitude / navaid_longitude
  // (v1 used ndb_*).
  const std::string sql = std::format(
      "SELECT icao_code, navaid_identifier, navaid_frequency, range, navaid_latitude, "
      "navaid_longitude FROM {}",
      table);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const Ident key(ColumnText(stmt, 1), ColumnText(stmt, 0));
    const std::optional<Coordinate> coord = ColumnCoord(stmt, 4, 5);
    if (!coord) {
      BF_LOG_WARN("dfd2: skipping '{}/{}' (no position)", key.ident, key.arinc424_icao_code);
      return;
    }
    if (!seen.Insert(key, *coord)) {
      return;
    }
    data.waypoints.push_back(Waypoint{key, *coord, WaypointKind::kNdb});
    data.navaid_details.push_back(
        NavaidDetail{key, WaypointKind::kNdb, 0, ColumnInt(stmt, 2), ColumnDouble(stmt, 3), 0.0});
  });
}

Result<void> LoadAirways(sqlite3* conn, NavData& data) {
  // Column names are the same as v1; only the table name differs.
  const std::string sql = std::format(
      "SELECT route_identifier, seqno, waypoint_identifier, icao_code, direction_restriction, "
      "flightlevel, minimum_altitude1, maximum_altitude, outbound_course, inbound_course, "
      "waypoint_description_code, waypoint_latitude, waypoint_longitude FROM {} ORDER BY "
      "route_identifier, seqno",
      kTblAirways);
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
  int prev_min_alt = 0, prev_max_alt = 0, prev_seqno = 0;
  std::optional<Coordinate> prev_coord;
  return ForEachRow(stmt, [&]() {
    const std::string route = ColumnText(stmt, 0);
    const std::string ident = ColumnText(stmt, 2);
    const std::string icao = ColumnText(stmt, 3);
    const std::optional<Coordinate> coord = ColumnCoord(stmt, 11, 12);
    const int seqno = ColumnInt(stmt, 1);
    // Column 2 ('E') of the waypoint description code marks End of Airway. Read
    // raw (untrimmed): trimming a blank column 1 would shift the byte offsets and
    // silently miss the 'E', reintroducing cross-instance phantom legs.
    const bool is_awy_end = HasCodeAt(ColumnTextRaw(stmt, 10), 1, kWptDescEndOfAirway);
    // An airway crossing an ARINC area boundary (e.g. A1 PAC -> EEU at IKELA) can
    // list the boundary fix twice under one seqno: once ending the first area's
    // portion (End of Airway set) and once starting the next. ORDER BY (route,
    // seqno) leaves the pair in either order, so chaining the rows naively gave a
    // zero-length self-loop and then broke the airway at the End flag, dropping
    // the following leg (DFD v2 2601: 711 such pairs, 557 lost edges). Fold the
    // pair into one point: the airway continues unless both rows end it, and the
    // outbound attributes come from the row that continues.
    if (have_prev && route == prev_route && seqno == prev_seqno && ident == prev_ident &&
        icao == prev_icao) {
      if (!is_awy_end) {
        prev_dir = ColumnText(stmt, 4);
        prev_level = ColumnText(stmt, 5);
        prev_min_alt = ColumnInt(stmt, 6);
        prev_max_alt = ColumnInt(stmt, 7);
      }
      prev_is_awy_end = prev_is_awy_end && is_awy_end;
      return;
    }
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
      data.airways.push_back(AirwayConnection{Ident(prev_ident, prev_icao), Ident(ident, icao), seg,
                                              prev_coord, coord});
    }
    prev_route = route;
    prev_coord = coord;
    prev_ident = ident;
    prev_icao = icao;
    prev_dir = ColumnText(stmt, 4);
    prev_level = ColumnText(stmt, 5);
    prev_min_alt = ColumnInt(stmt, 6);
    prev_max_alt = ColumnInt(stmt, 7);
    prev_seqno = seqno;
    prev_is_awy_end = is_awy_end;
    have_prev = true;
  });
}

Result<void> LoadAirports(sqlite3* conn, NavData& data) {
  const std::string sql = std::format(
      "SELECT icao_code, airport_identifier, airport_ref_latitude, airport_ref_longitude, "
      "elevation FROM {}",
      kTblAirports);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const std::optional<Coordinate> coord = ColumnCoord(stmt, 2, 3);
    if (!coord) {
      BF_LOG_WARN("dfd2: skipping airport '{}' (no position)", ColumnText(stmt, 1));
      return;
    }
    data.airports.push_back(
        Airport{ColumnText(stmt, 1), ColumnText(stmt, 0), *coord, ColumnInt(stmt, 4)});
  });
}

Result<void> LoadHoldings(sqlite3* conn, NavData& data) {
  const std::string sql = std::format(
      "SELECT region_code, icao_code, waypoint_identifier, inbound_holding_course, turn_direction, "
      "leg_length, leg_time, minimum_altitude, maximum_altitude, holding_speed FROM {}",
      kTblHoldings);
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
    h.turn_dir = EqualsCode(turn, kTurnLeft) ? kTurnLeft : kTurnRight;
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
  const std::string sql = std::format(
      "SELECT airport_identifier, msa_center, radius_limit, sector_bearing_1, sector_altitude_1, "
      "sector_bearing_2, sector_altitude_2, sector_bearing_3, sector_altitude_3, sector_bearing_4, "
      "sector_altitude_4, sector_bearing_5, sector_altitude_5, msa_center_icao_code FROM {}",
      kTblMsa);
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    MsaSector sector;
    sector.center = Ident{ColumnText(stmt, 1), ColumnText(stmt, 13)};
    sector.airport_icao = ColumnText(stmt, 0);
    const int radius = ColumnInt(stmt, 2);
    for (int i = 0; i < 5; ++i) {
      // v2 stores sector bearings as REAL in tens of degrees (26.5 = 265 deg,
      // matching earth_msa.dat and v1); reading them as int truncated LSZH's
      // 265/40/100/170 to 26/4/10/17.
      const int bearing = static_cast<int>(std::lround(ColumnDouble(stmt, 3 + i * 2) * 10.0));
      const int alt = ColumnInt(stmt, 4 + i * 2);
      if (bearing == 0 && alt == 0) {
        break;
      }
      // DFD writes a north sector boundary as 360; earth_msa.dat as 000.
      sector.arcs.push_back(MsaArc{bearing % 360, alt, radius});
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
  const std::string sql = std::format(
      "SELECT starting_latitude, starting_longitude, quadrant_code, mora01, mora02, mora03, "
      "mora04, mora05, mora06, mora07, mora08, mora09, mora10, mora11, mora12, mora13, mora14, "
      "mora15, mora16, mora17, mora18, mora19, mora20, mora21, mora22, mora23, mora24, mora25, "
      "mora26, mora27, mora28, mora29, mora30 FROM {}",
      kTblGridMora);
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
    const std::optional<int> lat = ParseHemiCoord(ColumnText(stmt, 0));
    const std::optional<int> lon0 = ParseHemiCoord(ColumnText(stmt, 1));
    if (!lat || !lon0) {
      BF_LOG_WARN("dfd2: skipping MORA row with malformed start '{}'/'{}'", ColumnText(stmt, 0),
                  ColumnText(stmt, 1));
      return;
    }
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
        data.mora.SetCell(*lat, *lon0 + i, static_cast<int16_t>(value));
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
  int wpt_desc = 18;     // waypoint_description_code (ARINC 424, 4 chars)
  int wp_lat = 19;       // waypoint_latitude
  int wp_lon = 20;       // waypoint_longitude
};

// SQL for one procedure table, in ProcCols column order.
std::string ProcSql(std::string_view table, bool single_airport) {
  // route_type is part of the flush key, so it must be in the ORDER BY to give
  // the partitioning a total order: without it, rows sharing (name, transition)
  // but differing in route_type could interleave and split one procedure across
  // two records (the SQL engine is free to return same-key rows in any order).
  std::string sql = std::string(
                        "SELECT airport_identifier, procedure_identifier, route_type, "
                        "transition_identifier, seqno, waypoint_identifier, waypoint_icao_code, "
                        "path_termination, course, course_flag, distance_time, "
                        "route_distance_holding_distance_time, altitude_description, altitude1, "
                        "altitude2, rnp, turn_direction, speed_limit, "
                        "waypoint_description_code, waypoint_latitude, waypoint_longitude "
                        "FROM ") +
                    std::string(table);
  if (single_airport) {
    sql += " WHERE airport_identifier = ?";
  }
  sql +=
      " ORDER BY airport_identifier, procedure_identifier, transition_identifier, "
      "route_type, seqno";
  return sql;
}

// Emit one Procedure leg from a row.
void AppendLeg(sqlite3_stmt* stmt, const ProcCols& c, Procedure& proc,
               std::optional<int>& prev_seqno) {
  const std::string wp_ident = ColumnText(stmt, c.wp_ident);
  Result<FixedIdent> fixed = FixedIdent::FromParts(wp_ident, ColumnText(stmt, c.wp_icao));
  if (!fixed) {
    BF_LOG_WARN("dfd2: skipping procedure leg with ident '{}': {}", wp_ident,
                fixed.error().message);
    return;
  }
  ProcedureLeg leg;
  leg.fix = std::move(fixed).value();
  leg.path_term = ParsePathTerminator(ColumnText(stmt, c.path_term));
  // course_flag 'T' marks a true course (true-referenced airspace); keep it as
  // published and flag it rather than converting with the airport variation,
  // so all loaders agree (X-Plane CIFP carries no variation to convert with).
  leg.course_deg = static_cast<float>(ColumnDouble(stmt, c.course));
  leg.course_true = EqualsCode(ColumnText(stmt, c.course_flag), kCourseFlagTrue);
  // v2 distance: distance_time holds the value, route_distance_... holds the flag.
  const std::string dflag = ColumnText(stmt, c.dist_flag);
  if (EqualsCode(dflag, kDistTimeFlagDistance)) {
    leg.distance_nm = static_cast<float>(ColumnDouble(stmt, c.dist_value));
  }
  leg.set_alt(ParseAltConstraint(ColumnText(stmt, c.alt_desc), ColumnInt(stmt, c.alt1),
                                 ColumnInt(stmt, c.alt2)));
  const double rnp = ColumnDouble(stmt, c.rnp);
  // rnp_centinm and speed_limit_kt are uint16_t; clamp the narrowing so an
  // out-of-range source value (negative, or beyond 65535) stays in range
  // instead of silently wrapping. Matches dfd1's FillProcExtras.
  const long rnp_centi = rnp > 0.0 ? std::lround(rnp * kCentiScale) : 0;
  leg.rnp_centinm = static_cast<uint16_t>(
      std::clamp<long>(rnp_centi, 0, static_cast<long>(std::numeric_limits<uint16_t>::max())));
  const std::string turn = ColumnText(stmt, c.turn_dir);
  leg.turn_dir = EqualsCode(turn, kTurnLeft)    ? kTurnLeft
                 : EqualsCode(turn, kTurnRight) ? kTurnRight
                                                : '\0';
  leg.speed_limit_kt = static_cast<uint16_t>(std::clamp(
      ColumnInt(stmt, c.speed_limit), 0, static_cast<int>(std::numeric_limits<uint16_t>::max())));
  leg.is_mapt = IsMaptDesc(ColumnText(stmt, c.wpt_desc));
  if (const std::optional<Coordinate> coord = ColumnCoord(stmt, c.wp_lat, c.wp_lon)) {
    leg.set_fix_coord(*coord);
  }
  proc.legs.push_back(std::move(leg));
  // Consume the seqno column (previously relied on only by ORDER BY) to validate
  // that legs arrive non-decreasing within a procedure -- the order the
  // partitioning into procedures depends on. A decrease means a query that dropped
  // the intended ORDER BY or corrupt source rows; warn rather than silently
  // mispartition. prev_seqno is reset (nullopt) at each procedure flush, so the
  // first leg of every procedure has no predecessor to compare against.
  const int seqno = ColumnInt(stmt, c.seqno);
  if (prev_seqno && seqno < *prev_seqno) {
    BF_LOG_WARN("dfd2: procedure {}/{}/{} leg seqno decreased ({} after {})",
                ColumnText(stmt, c.airport), ColumnText(stmt, c.proc),
                ColumnText(stmt, c.transition), seqno, *prev_seqno);
  }
  prev_seqno = seqno;
}

// Full-table scan appending per-airport procedures into `out`.
Result<void> LoadProcTable(sqlite3* conn, std::string_view table, ProcedureType type,
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
  std::optional<int> prev_seqno;  // reset per procedure; consumed by AppendLeg's order check

  auto flush_proc = [&]() {
    if (have_current && !current.legs.empty()) {
      cifp.procedures.push_back(std::move(current));
    }
    current = Procedure{};
    current.type = type;
    have_current = false;
    prev_seqno.reset();
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
    } else if (have_current && (name != cur_name || trans != cur_trans || route != cur_route)) {
      flush_proc();
    }
    if (!have_current) {
      cur_name = name;
      cur_trans = trans;
      cur_route = route;
      current.name = name;
      current.transition_ident = trans;
      if (trans.rfind(kRunwayPrefix, 0) == 0) {
        current.runway = trans;
      } else if (type == ProcedureType::kApproach) {
        current.runway = ApproachRunwayFromName(name);
      }
      current.route_type = ParseRouteTypeToken(route);
      have_current = true;
    }
    AppendLeg(stmt, c, current, prev_seqno);
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
  const std::string sql = std::format(
      "SELECT airport_identifier, runway_identifier, runway_latitude, runway_longitude, "
      "landing_threshold_elevation FROM {} ORDER BY airport_identifier, runway_identifier",
      kTblRunways);
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
// The airport's procedures and runways, nullopt if it has neither, or an Error
// if a query fails (a missing table or a step error is a read failure, not an
// airport without procedures).
Result<std::optional<CifpData>> LoadAirportProcedures(sqlite3* conn, const std::string& airport) {
  CifpData cifp;
  const std::pair<std::string_view, ProcedureType> kTables[] = {
      {kTblSids, ProcedureType::kSid},
      {kTblStars, ProcedureType::kStar},
      {kTblIaps, ProcedureType::kApproach},
  };
  for (const auto& [table, type] : kTables) {
    Result<SqliteStmt> s = Prepare(conn, ProcSql(table, /*single_airport=*/true));
    if (!s) {
      return Result<std::optional<CifpData>>::Err(std::move(s).error());
    }
    sqlite3_stmt* stmt = s.value().get();
    sqlite3_bind_text(stmt, 1, airport.c_str(), -1, SQLITE_TRANSIENT);
    const ProcCols c;
    Procedure current;
    current.type = type;
    bool have_current = false;
    std::string cur_name, cur_trans, cur_route;
    std::optional<int> prev_seqno;  // reset per procedure; consumed by AppendLeg's order check
    auto flush_current = [&]() {
      if (have_current && !current.legs.empty()) {
        cifp.procedures.push_back(std::move(current));
      }
      current = Procedure{};
      current.type = type;
      have_current = false;
      prev_seqno.reset();
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
        if (trans.rfind(kRunwayPrefix, 0) == 0) {
          current.runway = trans;
        } else if (type == ProcedureType::kApproach) {
          current.runway = ApproachRunwayFromName(name);
        }
        current.route_type = ParseRouteTypeToken(route);
        have_current = true;
      }
      AppendLeg(stmt, c, current, prev_seqno);
    });
    if (!rows) {
      return Result<std::optional<CifpData>>::Err(std::move(rows).error());
    }
    flush_current();
  }
  Result<SqliteStmt> rs = Prepare(
      conn, std::format("SELECT runway_identifier, runway_latitude, runway_longitude, "
                        "landing_threshold_elevation FROM {} WHERE airport_identifier = ? ORDER BY "
                        "runway_identifier",
                        kTblRunways));
  if (!rs) {
    return Result<std::optional<CifpData>>::Err(std::move(rs).error());
  }
  {
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
      return Result<std::optional<CifpData>>::Err(std::move(rows).error());
    }
  }
  if (cifp.procedures.empty() && cifp.runways.empty()) {
    return Result<std::optional<CifpData>>::Ok(std::nullopt);
  }
  return Result<std::optional<CifpData>>::Ok(std::move(cifp));
}

}  // namespace

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
  const std::string cycle_sql = std::format("SELECT cycle FROM {} LIMIT 1", kTblHeader);
  Result<SqliteStmt> s = Prepare(conn.value(), cycle_sql);
  if (s) {
    Result<bool> row = Step(s.value().get());
    if (!row) {
      return Result<NavData>::Err(row.error());
    }
    if (row.value()) {
      const std::string cyc = ColumnText(s.value().get(), 0);
      unsigned int cycle = 0;
      // A non-numeric (or partially numeric) cycle leaves 0 ("no AIRAC
      // provenance") rather than a half-parsed value. (Same as dfd1.)
      const auto [ptr, ec] = std::from_chars(cyc.data(), cyc.data() + cyc.size(), cycle);
      if (ec == std::errc{} && ptr == cyc.data() + cyc.size()) {
        data.cycle = cycle;
      }
    }
  }

  // Each per-table loader returns Result<void>; a step error (corruption, IO)
  // propagates as a load failure rather than silently producing partial data.
  PointDeduper seen(kLoaderName);
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
  // first-wins same-point merge; a same-key terminal fix elsewhere is kept as its
  // own vertex. (Same as dfd1.)
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
  std::vector<AirportProcedureData> out;
  Result<void> load = Result<void>::Ok();
  load = LoadProcTable(conn.value(), kTblSids, ProcedureType::kSid, out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  load = LoadProcTable(conn.value(), kTblStars, ProcedureType::kStar, out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  load = LoadProcTable(conn.value(), kTblIaps, ProcedureType::kApproach, out);
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
  // MergeRunways appends runway-only airports in hash-map order; sort so the
  // output (and the .bfdb built from it) is reproducible run to run.
  std::sort(merged.begin(), merged.end(),
            [](const AirportProcedureData& a, const AirportProcedureData& b) {
              return a.first < b.first;
            });
  return Result<std::vector<AirportProcedureData>>::Ok(std::move(merged));
}

Result<std::optional<CifpData>> Dfd2Loader::LoadProcedure(const std::string& source_dir,
                                                          const std::string& icao) const {
  Result<std::string> db_path = FindV2Db(source_dir);
  if (!db_path) {
    return Result<std::optional<CifpData>>::Err(std::move(db_path).error());
  }
  Result<sqlite3*> conn = AcquireConn(kLoaderName, db_path.value());
  if (!conn) {
    return Result<std::optional<CifpData>>::Err(std::move(conn).error());
  }
  return LoadAirportProcedures(conn.value(), icao);
}

}  // namespace bf
