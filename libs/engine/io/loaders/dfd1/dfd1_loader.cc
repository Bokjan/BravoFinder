// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/dfd1/dfd1_loader.h"

#include <sqlite3.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
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

// DFD v1.0 source loader. All table/row -> domain mapping is private to this TU.
// Column access is positional against an explicit SELECT, so the table's native
// column order never matters. Units that differ from X-Plane CIFP (the reference
// loader) are called out inline.

constexpr std::string_view kLoaderName = "dfd1";

// Feet per flight level (100 ft per FL); DFD stores altitudes in feet, the
// graph works in flight levels.
constexpr int kFeetPerFlightLevel = 100;
// DFD encodes "no ceiling" as maximum_altitude = 99999; treated as unset.
constexpr int kUnknownAltitudeFt = 99999;

// Search source_dir for the v1 database (byte-identical copies under several
// names), in priority order, falling back to the first *.s3db found.
Result<std::string> FindV1Db(const std::string& source_dir) {
  namespace fs = std::filesystem;
  std::error_code ec;
  if (!fs::is_directory(source_dir, ec)) {
    return Result<std::string>::Err(
        Error(ErrorCode::kDataMissing, "source directory not found: " + source_dir));
  }
  const char* kPreferred[] = {"navdb.s3db", "navdata.s3db", "e_dfd_PMDG.s3db"};
  for (const char* name : kPreferred) {
    fs::path p = fs::path(source_dir) / name;
    if (fs::exists(p, ec)) {
      return Result<std::string>::Ok(p.string());
    }
  }
  for (const fs::directory_entry& de : fs::directory_iterator(source_dir, ec)) {
    if (de.is_regular_file() && de.path().extension() == ".s3db") {
      return Result<std::string>::Ok(de.path().string());
    }
  }
  return Result<std::string>::Err(
      Error(ErrorCode::kDataMissing, "no DFD v1 .s3db under " + source_dir));
}

// Verify the DB is DFD v1.0 (tbl_header.version starts with "1."). A missing
// tbl_header (Prepare fails or no row) or a different version means the user
// likely pointed dfd1 at a v2 DB (or vice versa); give an actionable message.
Result<void> CheckV1Header(sqlite3* conn) {
  Result<SqliteStmt> stmt = Prepare(conn, "SELECT version FROM tbl_header LIMIT 1");
  std::string version;
  if (stmt) {
    Result<bool> row = Step(stmt.value().get());
    if (row && row.value()) {
      version = ColumnText(stmt.value().get(), 0);
    }
  }
  if (version.empty() || version.rfind("1.", 0) != 0) {
    return Result<void>::Err(Error(ErrorCode::kInvalidArgument,
                                   "not a DFD v1.0 database (tbl_header missing or wrong version); "
                                   "for DFD v2 use --loader dfd2"));
  }
  return Result<void>::Ok();
}

// Map a DFD navaid_class first character to a routable WaypointKind. See the
// plan: V=VOR/VOR-DME/VORTAC, D/T/M=DME/TACAN (kDme), I/N/P=ILS/MLS components
// (kOther, skipped), leading space = ILS/LOC component (kOther, skipped).
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

// --- Per-table loaders. Each runs one query and appends to `data`. ---

Result<void> LoadEnrouteWaypoints(sqlite3* conn, NavData& data, std::unordered_set<Ident>& seen) {
  // 0=icao_code 1=waypoint_identifier 2=lat 3=lon
  Result<SqliteStmt> s =
      Prepare(conn,
              "SELECT icao_code, waypoint_identifier, waypoint_latitude, waypoint_longitude "
              "FROM tbl_enroute_waypoints");
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  int skipped = 0;
  Result<void> result = ForEachRow(stmt, [&]() {
    std::string id = ColumnText(stmt, 1);
    if (id.size() > FixedIdent::kIdentCap) {
#ifndef NDEBUG
      std::fprintf(stderr,
                   "dfd1: skipping waypoint '%s' (ident too long for FixedIdent, %zu > %d)\n",
                   id.c_str(), id.size(), FixedIdent::kIdentCap);
#endif
      ++skipped;
      return;
    }
    const Ident key(id, ColumnText(stmt, 0));
    if (seen.insert(key).second) {
      data.waypoints.push_back(Waypoint{
          key, Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)}, WaypointKind::kFix});
    }
  });
  if (skipped > 0) {
#ifndef NDEBUG
    std::fprintf(stderr, "dfd1: %d waypoint(s) skipped (ident too long for FixedIdent)\n", skipped);
#endif
  }
  return result;
}

Result<void> LoadTerminalWaypoints(sqlite3* conn, NavData& data, std::unordered_set<Ident>& seen) {
  // 0=icao_code 1=waypoint_identifier 2=lat 3=lon
  // Use icao_code (the 2-char ICAO region), NOT region_code -- region_code is the
  // airport/heliport the terminal fix belongs to (e.g. "01OH", 4 chars), which is
  // not a region: keying by it mismatches how airways/procedures reference the fix
  // (by icao_code) and overflows FixedIdent::kRegionCap. Matches LoadEnrouteWaypoints.
  Result<SqliteStmt> s =
      Prepare(conn,
              "SELECT icao_code, waypoint_identifier, waypoint_latitude, waypoint_longitude "
              "FROM tbl_terminal_waypoints");
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  int skipped = 0;
  Result<void> result = ForEachRow(stmt, [&]() {
    std::string id = ColumnText(stmt, 1);
    if (id.size() > FixedIdent::kIdentCap) {
#ifndef NDEBUG
      std::fprintf(
          stderr,
          "dfd1: skipping terminal waypoint '%s' (ident too long for FixedIdent, %zu > %d)\n",
          id.c_str(), id.size(), FixedIdent::kIdentCap);
#endif
      ++skipped;
      return;
    }
    const Ident key(id, ColumnText(stmt, 0));
    if (seen.insert(key).second) {
      data.waypoints.push_back(Waypoint{
          key, Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)}, WaypointKind::kFix});
    }
  });
  if (skipped > 0) {
#ifndef NDEBUG
    std::fprintf(stderr, "dfd1: %d terminal waypoint(s) skipped (ident too long for FixedIdent)\n",
                 skipped);
#endif
  }
  return result;
}

Result<void> LoadVhfNavaids(sqlite3* conn, NavData& data, std::unordered_set<Ident>& seen) {
  // 0=icao_code 1=vor_identifier 2=vor_frequency 3=navaid_class 4=range
  // 5=station_declination 6=dme_elevation 7=vor_lat 8=vor_lon
  Result<SqliteStmt> s = Prepare(
      conn,
      "SELECT icao_code, vor_identifier, vor_frequency, navaid_class, range, "
      "station_declination, dme_elevation, vor_latitude, vor_longitude FROM tbl_vhfnavaids");
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
    // DFD vor_frequency is MHz (e.g. 115.9); NavaidDetail.freq_raw is MHz*100.
    const int freq_raw = static_cast<int>(std::lround(ColumnDouble(stmt, 2) * 100.0));
    data.navaid_details.push_back(NavaidDetail{key, kind, ColumnInt(stmt, 6), freq_raw,
                                               ColumnDouble(stmt, 4), ColumnDouble(stmt, 5)});
  });
}

Result<void> LoadNdbNavaids(sqlite3* conn, NavData& data, const char* table,
                            std::unordered_set<Ident>& seen) {
  // 0=icao_code 1=ndb_identifier 2=ndb_frequency 3=range 4=lat 5=lon
  const std::string sql = std::string(
                              "SELECT icao_code, ndb_identifier, ndb_frequency, range, "
                              "ndb_latitude, ndb_longitude FROM ") +
                          table;
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
    // NDB frequency is kHz; freq_raw stores kHz as-is.
    data.navaid_details.push_back(
        NavaidDetail{key, WaypointKind::kNdb, 0, ColumnInt(stmt, 2), ColumnDouble(stmt, 3), 0.0});
  });
}

Result<void> LoadAirways(sqlite3* conn, NavData& data) {
  // One row per waypoint on an airway; consecutive same-route_identifier rows (by
  // seqno) form segments. 0=route_identifier 1=seqno 2=waypoint_identifier
  // 3=icao_code 4=direction_restriction 5=flightlevel 6=minimum_altitude1
  // 7=maximum_altitude 8=outbound_course 9=inbound_course
  // 10=waypoint_description_code
  Result<SqliteStmt> s =
      Prepare(conn,
              "SELECT route_identifier, seqno, waypoint_identifier, icao_code, "
              "direction_restriction, flightlevel, minimum_altitude1, maximum_altitude, "
              "outbound_course, inbound_course, waypoint_description_code "
              "FROM tbl_enroute_airways ORDER BY route_identifier, seqno");
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  bool have_prev = false;
  // route_identifier is NOT unique per physical airway: a single identifier (e.g.
  // "V105") can carry several geographically disjoint airway strings that share
  // the name, distinguished only by icao_code (US K2 / China ZB-ZH / India VA).
  // ARINC 424 waypoint_description_code column 2 == 'E' ("End of Airway") flags the
  // LAST fix of each string, so we must break the prev->current chain there --
  // otherwise the last US fix would be joined to the first China fix, forging a
  // ~5400 nm cross-ocean phantom leg. icao_code changes cannot be used (a real
  // airway crosses regions); the description code is the authoritative boundary.
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
      AirwaySegment seg;
      seg.name = prev_route;
      seg.direction = ParseDirection(prev_dir);
      seg.level = ParseAirwayLevel(prev_level);  // 'H'/'L'/'B'
      // DFD altitudes are FEET; base_fl/top_fl are flight levels (feet/100).
      // DFD encodes "no ceiling" as maximum_altitude=99999 -> top_fl=999. That is
      // a very high finite band, NOT the base_fl==0 && top_fl==0 "no recorded
      // band" sentinel AltitudeBandConstraint exempts; harmless since no query
      // cruises above FL999.
      seg.base_fl = prev_min_alt / kFeetPerFlightLevel;
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
    // Column 2 ('E') of the waypoint description code marks End of Airway; the next
    // same-route row then starts a fresh string and must not be chained to this fix.
    // Read raw (untrimmed): trimming a blank column 1 would shift the byte offsets
    // and silently miss the 'E', reintroducing cross-instance phantom legs.
    const std::string desc = ColumnTextRaw(stmt, 10);
    prev_is_awy_end = desc.size() > 1 && desc[1] == 'E';
    have_prev = true;
  });
}

Result<void> LoadAirports(sqlite3* conn, NavData& data) {
  // 0=icao_code 1=airport_identifier 2=lat 3=lon 4=elevation
  Result<SqliteStmt> s = Prepare(conn,
                                 "SELECT icao_code, airport_identifier, airport_ref_latitude, "
                                 "airport_ref_longitude, elevation FROM tbl_airports");
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

Result<void> LoadHoldings(sqlite3* conn, NavData& data) {
  // 0=region_code 1=icao_code 2=waypoint_identifier 3=inbound_holding_course
  // 4=turn_direction 5=leg_length 6=leg_time 7=minimum_altitude 8=maximum_altitude
  // 9=holding_speed
  Result<SqliteStmt> s =
      Prepare(conn,
              "SELECT region_code, icao_code, waypoint_identifier, inbound_holding_course, "
              "turn_direction, leg_length, leg_time, minimum_altitude, maximum_altitude, "
              "holding_speed FROM tbl_holdings");
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    HoldFix h;
    h.fix = Ident(ColumnText(stmt, 2), ColumnText(stmt, 1));  // region = icao_code
    h.airport_icao = ColumnText(stmt, 0);                     // region_code = airport/ENRT
    h.inbound_course = ColumnDouble(stmt, 3);
    const std::string turn = ColumnText(stmt, 4);
    // Hold turn direction defaults to right ('R') when absent/unknown, matching
    // the X-Plane earth_hold parse. (Leg turn_dir elsewhere uses '\0' for none.)
    h.turn_dir = (turn == "L") ? 'L' : 'R';
    h.leg_dist_nm = ColumnDouble(stmt, 5);
    h.leg_time_min = ColumnDouble(stmt, 6);
    h.min_alt_ft = ColumnInt(stmt, 7);
    // DFD uses 99999 for "no upper limit"; HoldFix uses 0 (matching X-Plane).
    const int max_alt = ColumnInt(stmt, 8);
    h.max_alt_ft = (max_alt >= kUnknownAltitudeFt) ? 0 : max_alt;
    h.speed_limit_kt = ColumnInt(stmt, 9);
    data.hold_fixes.push_back(std::move(h));
  });
}

Result<void> LoadMsa(sqlite3* conn, NavData& data) {
  // 0=airport_identifier 1=msa_center 2=msa_center_latitude 3=msa_center_longitude
  // 4=radius_limit 5..6=sector1(bearing,alt) 7..8=sector2 ...
  Result<SqliteStmt> s =
      Prepare(conn,
              "SELECT airport_identifier, msa_center, msa_center_latitude, "
              "msa_center_longitude, radius_limit, "
              "sector_bearing_1, sector_altitude_1, sector_bearing_2, sector_altitude_2, "
              "sector_bearing_3, sector_altitude_3, sector_bearing_4, sector_altitude_4, "
              "sector_bearing_5, sector_altitude_5 FROM tbl_airport_msa");
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    MsaSector sector;
    // center latitude/longitude are available but MsaSector.center is an Ident
    // with no coordinate slot (matching X-Plane, which also discards them); the
    // center is resolved by name from the waypoint/navaid set.
    sector.center = Ident{ColumnText(stmt, 1), {}};
    sector.airport_icao = ColumnText(stmt, 0);
    const int radius = ColumnInt(stmt, 4);
    for (int i = 0; i < 5; ++i) {
      const int bearing = ColumnInt(stmt, 5 + i * 2);
      const int alt = ColumnInt(stmt, 6 + i * 2);
      if (bearing == 0 && alt == 0) {
        break;  // unused sector slot
      }
      sector.arcs.push_back(MsaArc{bearing, alt, radius});
    }
    if (!sector.arcs.empty()) {
      data.msa.push_back(std::move(sector));
    }
  });
}

Result<void> LoadGridMora(sqlite3* conn, NavData& data) {
  // v1: starting_latitude/longitude are INTEGER. mora01..mora30 are TEXT(3) ("010"
  // or "UNK"). Each row covers 30 one-degree cells of longitude from lon0.
  Result<SqliteStmt> s =
      Prepare(conn,
              "SELECT starting_latitude, starting_longitude, "
              "mora01, mora02, mora03, mora04, mora05, mora06, mora07, mora08, mora09, mora10, "
              "mora11, mora12, mora13, mora14, mora15, mora16, mora17, mora18, mora19, mora20, "
              "mora21, mora22, mora23, mora24, mora25, mora26, mora27, mora28, mora29, mora30 "
              "FROM tbl_grid_mora");
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  return ForEachRow(stmt, [&]() {
    const int lat = ColumnInt(stmt, 0);
    const int lon0 = ColumnInt(stmt, 1);
    for (int i = 0; i < 30; ++i) {
      const std::string v = ColumnText(stmt, 2 + i);
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

}  // namespace

Result<NavData> Dfd1Loader::LoadNavData(const std::string& source_dir) const {
  Result<std::string> db_path = FindV1Db(source_dir);
  if (!db_path) {
    return Result<NavData>::Err(std::move(db_path).error());
  }
  Result<sqlite3*> conn = AcquireConn(kLoaderName, db_path.value());
  if (!conn) {
    return Result<NavData>::Err(std::move(conn).error());
  }
  Result<void> header = CheckV1Header(conn.value());
  if (!header) {
    return Result<NavData>::Err(std::move(header).error());
  }

  NavData data;
  // AIRAC cycle from tbl_header.current_airac (e.g. "2601").
  Result<SqliteStmt> s = Prepare(conn.value(), "SELECT current_airac FROM tbl_header LIMIT 1");
  if (s) {
    Result<bool> row = Step(s.value().get());
    if (!row) {
      return Result<NavData>::Err(row.error());
    }
    if (row.value()) {
      const std::string cyc = ColumnText(s.value().get(), 0);
      unsigned int cycle = 0;
      // A non-numeric (or partially numeric) current_airac leaves cycle at 0,
      // which means "no AIRAC provenance", rather than a half-parsed value.
      // Check from_chars's result instead of discarding it.
      const auto [ptr, ec] = std::from_chars(cyc.data(), cyc.data() + cyc.size(), cycle);
      if (ec == std::errc{} && ptr == cyc.data() + cyc.size()) {
        data.cycle = cycle;
      }
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
  load = LoadNdbNavaids(conn.value(), data, "tbl_enroute_ndbnavaids", seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  load = LoadNdbNavaids(conn.value(), data, "tbl_terminal_ndbnavaids", seen);
  if (!load) {
    return Result<NavData>::Err(load.error());
  }
  // Terminal waypoints load LAST: keyed by icao_code they can now share an
  // (ident, region) with an enroute fix or navaid, and the shared `seen` set is
  // first-wins -- letting the canonical enroute/navaid entries take priority.
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

namespace {

// Column layout for the three procedure tables (v1), as positional indices into
// the SELECT defined in LoadProcTable. v2 reorders/renames some columns, so
// Dfd2Loader defines its own layout; here is v1's.
struct ProcCols {
  int airport = 0;       // airport_identifier
  int proc = 1;          // procedure_identifier
  int route_type = 2;    // route_type (numeric for SID/STAR, alpha for IAP)
  int transition = 3;    // transition_identifier
  int seqno = 4;         // seqno
  int wp_ident = 5;      // waypoint_identifier
  int wp_icao = 6;       // waypoint_icao_code
  int path_term = 7;     // path_termination
  int course = 8;        // magnetic_course
  int dist_value = 9;    // route_distance_holding_distance_time (REAL)
  int dist_flag = 10;    // distance_time (TEXT 'D'/'T'/blank)
  int alt_desc = 11;     // altitude_description
  int alt1 = 12;         // altitude1
  int alt2 = 13;         // altitude2
  int rnp = 14;          // rnp (DOUBLE, nautical miles)
  int turn_dir = 15;     // turn_direction ('L'/'R')
  int speed_limit = 16;  // speed_limit (knots)
};

// Fill the leg-level RNP/turn/speed fields shared by both v1 procedure paths
// (full-table scan and on-demand). rnp is a DOUBLE in nautical miles stored as
// hundredths; turn_direction is 'L'/'R' text; speed_limit is an integer in knots.
void FillProcExtras(sqlite3_stmt* stmt, const ProcCols& c, ProcedureLeg& leg) {
  const double rnp = ColumnDouble(stmt, c.rnp);
  // rnp_centinm and speed_limit_kt are uint16_t. Guard the narrowing so an
  // out-of-range DFD value (negative, or beyond 65535) clamps to the field range
  // instead of silently wrapping. Real data is well within bounds (RNP <= ~15 nm,
  // speeds <= ~350 kt); this only hardens against a corrupt/unexpected source row.
  const long rnp_centi = rnp > 0.0 ? std::lround(rnp * 100.0) : 0;
  leg.rnp_centinm = static_cast<uint16_t>(std::clamp<long>(rnp_centi, 0, 65535));
  const std::string turn = ColumnText(stmt, c.turn_dir);
  leg.turn_dir = (turn == "L") ? 'L' : (turn == "R") ? 'R' : '\0';
  const int speed = ColumnInt(stmt, c.speed_limit);
  leg.speed_limit_kt = static_cast<uint16_t>(std::clamp(speed, 0, 65535));
}

// Append legs/runways from one procedure table into `out` (per-airport). `type`
// is fixed by the caller (sids->kSid, etc.). One full-table scan per table: the
// rows are ordered by (airport, name, transition, route_type, seqno), so a
// change in (airport, name, transition, route_type) flushes the current
// Procedure, and a change in airport emits the previous airport's accumulated
// CifpData. This avoids per-airport `Prepare` (~7k airports x 3 tables).
Result<void> LoadProcTable(sqlite3* conn, const char* table, ProcedureType type,
                           std::vector<AirportProcedureData>& out) {
  // route_type is part of the flush key, so it must be in the ORDER BY to give
  // the partitioning a total order: without it, rows sharing (name, transition)
  // but differing in route_type could interleave and split one procedure across
  // two records (the SQL engine is free to return same-key rows in any order).
  const std::string sql =
      std::string(
          "SELECT airport_identifier, procedure_identifier, route_type, "
          "transition_identifier, seqno, waypoint_identifier, waypoint_icao_code, "
          "path_termination, magnetic_course, route_distance_holding_distance_time, "
          "distance_time, altitude_description, altitude1, altitude2, rnp, turn_direction, "
          "speed_limit FROM ") +
      table +
      " ORDER BY airport_identifier, procedure_identifier, "
      "transition_identifier, route_type, seqno";
  Result<SqliteStmt> s = Prepare(conn, sql);
  if (!s) {
    return Result<void>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  const ProcCols c;

  Procedure current;
  current.type = type;
  bool have_current = false;
  std::string cur_airport, cur_name, cur_trans, cur_route;
  CifpData cifp;  // accumulated for cur_airport

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
        current.runway = trans;  // runway transition
      }
      // route_type is numeric for SID/STAR, alpha for IAP; keep numeric value as
      // provenance, 0 when not a number. Pure provenance -- never affects routing.
      // from_chars (no exceptions) since IAP route_type is non-numeric.
      int rt = 0;
      const char* d = route.data();
      const char* e = d + route.size();
      std::from_chars(d, e, rt);
      current.route_type = rt;
      have_current = true;
    }
    ProcedureLeg leg;
    // FixedIdent::kIdentCap = 7; skip legs whose waypoint ident is too long.
    std::string wp_ident = ColumnText(stmt, c.wp_ident);
    if (wp_ident.size() > FixedIdent::kIdentCap) {
#ifndef NDEBUG
      std::fprintf(
          stderr,
          "dfd1: skipping procedure leg with ident '%s' (too long for FixedIdent, %zu > %d)\n",
          wp_ident.c_str(), wp_ident.size(), FixedIdent::kIdentCap);
#endif
      return;
    }
    leg.fix = FixedIdent::FromParts(wp_ident, ColumnText(stmt, c.wp_icao));
    leg.path_term = ParsePathTerminator(ColumnText(stmt, c.path_term));
    leg.course_deg = ColumnDouble(stmt, c.course);  // DFD: degrees (not tenths)
    // distance_flag 'D'=distance in nm, 'T'=time (no field for it), blank=none.
    const std::string flag = ColumnText(stmt, c.dist_flag);
    if (flag == "D") {
      leg.distance_nm = ColumnDouble(stmt, c.dist_value);
    }
    leg.alt = ParseAltConstraint(ColumnText(stmt, c.alt_desc), ColumnInt(stmt, c.alt1),
                                 ColumnInt(stmt, c.alt2));
    FillProcExtras(stmt, c, leg);
    current.legs.push_back(std::move(leg));
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  // Flush the last airport's accumulated procedures.
  flush_airport();
  return Result<void>::Ok();
}

// All runways, indexed by airport, so LoadProcedures can merge them into each
// airport's CifpData in one pass (no per-airport query).
Result<std::unordered_map<std::string, std::vector<Runway>>> LoadAllRunways(sqlite3* conn) {
  std::unordered_map<std::string, std::vector<Runway>> by_airport;
  Result<SqliteStmt> s = Prepare(conn,
                                 "SELECT airport_identifier, runway_identifier, runway_latitude, "
                                 "runway_longitude, landing_threshold_elevation FROM tbl_runways "
                                 "ORDER BY airport_identifier");
  if (!s) {
    return Result<std::unordered_map<std::string, std::vector<Runway>>>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&]() {
    Runway rwy;
    rwy.ident = ColumnText(stmt, 1);  // already "RW04L"-prefixed in DFD
    rwy.threshold = Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)};
    rwy.elevation_ft = ColumnInt(stmt, 4);
    by_airport[ColumnText(stmt, 0)].push_back(std::move(rwy));
  });
  if (!rows) {
    return Result<std::unordered_map<std::string, std::vector<Runway>>>::Err(rows.error());
  }
  return Result<std::unordered_map<std::string, std::vector<Runway>>>::Ok(std::move(by_airport));
}

// Merge each airport's runways into its CifpData entry, in place. Runways go
// only to airports that already have procedures (an airport with procedures but
// no runways, or vice versa, is handled by the procedure scan).
void MergeRunways(std::vector<AirportProcedureData>& out,
                  std::unordered_map<std::string, std::vector<Runway>>& runways) {
  for (AirportProcedureData& ap : out) {
    auto it = runways.find(ap.first);
    if (it != runways.end()) {
      ap.second.runways = std::move(it->second);
      runways.erase(it);
    }
  }
  // Airports with runways but no procedures: keep them (a runway is useful as a
  // route endpoint even without SID/STAR/approach procedures).
  for (auto& [airport, rwys] : runways) {
    if (!rwys.empty()) {
      CifpData cifp;
      cifp.runways = std::move(rwys);
      out.emplace_back(airport, std::move(cifp));
    }
  }
}

// Load one airport's procedures + runways on demand (LoadProcedure path). A
// single per-airport prepare per table is fine here: this is the on-demand path,
// called once per airport the routing touches. Returns nullopt when the airport
// has no procedures/runways; a SQLite step error degrades to nullopt too (this
// path is best-effort -- the full LoadProcedures path propagates such errors).
std::optional<CifpData> LoadAirportProcedures(sqlite3* conn, const std::string& airport) {
  CifpData cifp;
  for (const auto& [table, type] : {std::pair{"tbl_sids", ProcedureType::kSid},
                                    {"tbl_stars", ProcedureType::kStar},
                                    {"tbl_iaps", ProcedureType::kApproach}}) {
    // route_type is part of the flush key, so include it in the ORDER BY (see
    // LoadProcTable for why a total order matters).
    const std::string sql =
        std::string(
            "SELECT airport_identifier, procedure_identifier, route_type, "
            "transition_identifier, seqno, waypoint_identifier, waypoint_icao_code, "
            "path_termination, magnetic_course, route_distance_holding_distance_time, "
            "distance_time, altitude_description, altitude1, altitude2, rnp, turn_direction, "
            "speed_limit FROM ") +
        table +
        " WHERE airport_identifier = ? ORDER BY procedure_identifier, "
        "transition_identifier, route_type, seqno";
    Result<SqliteStmt> s = Prepare(conn, sql);
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
      ProcedureLeg leg;
      std::string wp_ident2 = ColumnText(stmt, c.wp_ident);
      if (wp_ident2.size() > FixedIdent::kIdentCap) {
#ifndef NDEBUG
        std::fprintf(
            stderr,
            "dfd1: skipping procedure leg with ident '%s' (too long for FixedIdent, %zu > %d)\n",
            wp_ident2.c_str(), wp_ident2.size(), FixedIdent::kIdentCap);
#endif
        return;
      }
      leg.fix = FixedIdent::FromParts(wp_ident2, ColumnText(stmt, c.wp_icao));
      leg.path_term = ParsePathTerminator(ColumnText(stmt, c.path_term));
      leg.course_deg = ColumnDouble(stmt, c.course);
      const std::string flag = ColumnText(stmt, c.dist_flag);
      if (flag == "D") {
        leg.distance_nm = ColumnDouble(stmt, c.dist_value);
      }
      leg.alt = ParseAltConstraint(ColumnText(stmt, c.alt_desc), ColumnInt(stmt, c.alt1),
                                   ColumnInt(stmt, c.alt2));
      FillProcExtras(stmt, c, leg);
      current.legs.push_back(std::move(leg));
    });
    if (!rows) {
      return std::nullopt;  // step error: treat as load failure for this airport
    }
    flush_current();
  }
  // Runways for this airport.
  Result<SqliteStmt> rs =
      Prepare(conn,
              "SELECT runway_identifier, runway_latitude, runway_longitude, "
              "landing_threshold_elevation FROM tbl_runways WHERE airport_identifier = ?");
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

Result<std::vector<AirportProcedureData>> Dfd1Loader::LoadProcedures(
    const std::string& source_dir) const {
  Result<std::string> db_path = FindV1Db(source_dir);
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
  // "for DFD v2 use --loader dfd2" error instead of an obscure kParseError.
  if (Result<void> header = CheckV1Header(conn.value()); !header) {
    return Result<std::vector<AirportProcedureData>>::Err(std::move(header).error());
  }
  std::vector<AirportProcedureData> out;
  Result<void> load = Result<void>::Ok();
  load = LoadProcTable(conn.value(), "tbl_sids", ProcedureType::kSid, out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  load = LoadProcTable(conn.value(), "tbl_stars", ProcedureType::kStar, out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  load = LoadProcTable(conn.value(), "tbl_iaps", ProcedureType::kApproach, out);
  if (!load) {
    return Result<std::vector<AirportProcedureData>>::Err(load.error());
  }
  // The three tables were scanned independently, so `out` is grouped per table,
  // not globally ordered by airport. Sort so each airport's procedures are
  // contiguous, then merge runways.
  std::sort(out.begin(), out.end(),
            [](const AirportProcedureData& a, const AirportProcedureData& b) {
              return a.first < b.first;
            });
  // The same airport may appear up to three times (once per table); merge them.
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

std::optional<CifpData> Dfd1Loader::LoadProcedure(const std::string& source_dir,
                                                  const std::string& icao) const {
  Result<std::string> db_path = FindV1Db(source_dir);
  if (!db_path) {
    return std::nullopt;
  }
  Result<sqlite3*> conn = AcquireConn(kLoaderName, db_path.value());
  if (!conn) {
    return std::nullopt;
  }
  return LoadAirportProcedures(conn.value(), icao);
}

}  // namespace bf
