// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/fenix/fenix_loader.h"

#include <sqlite3.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <format>
#include <limits>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

#include "core/base/log.h"
#include "core/domain/airport.h"
#include "core/domain/airway.h"
#include "core/domain/arinc_codes.h"
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
#include "io/loaders/loader_constants.h"
#include "io/loaders/sqlite_util.h"
#include "io/nav_data.h"

namespace bf {
namespace {

// Fenix A320 SQLite table names (PascalCase / config). Named so SQL fragments
// cannot drift by typo; not shared with DFD (different schema entirely).
constexpr std::string_view kTblConfig = "config";
constexpr std::string_view kTblWaypointLookup = "WaypointLookup";
constexpr std::string_view kTblNavaids = "Navaids";
constexpr std::string_view kTblWaypoints = "Waypoints";
constexpr std::string_view kTblAirways = "Airways";
constexpr std::string_view kTblAirwayLegs = "AirwayLegs";
constexpr std::string_view kTblAirports = "Airports";
constexpr std::string_view kTblHoldings = "Holdings";
constexpr std::string_view kTblGridMora = "GridMora";
constexpr std::string_view kTblTerminalLegs = "TerminalLegs";
constexpr std::string_view kTblTerminals = "Terminals";
constexpr std::string_view kTblRunways = "Runways";

constexpr std::string_view kLoaderName = "fenix";

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

// Fenix Navaids.Type → WaypointKind.  The authoritative mapping is the
// NavaidTypes table (Type column + Desc column).  The Type column is TEXT;
// sqlite3_column_int() converts via SQLite type affinity.
//
//   Type  NavaidTypes.Name        → WaypointKind   Rationale
//     1   VOR                     → kVor           VOR / VOR-DME
//     2   VORTAC                  → kVor           VOR primary
//     3   TACAN                   → kDme           DME / TACAN
//     4   VOR-DME                 → kVor           VOR / VOR-DME
//     5   NDB                     → kNdb           NDB
//     7   NDB-DME                 → kNdb           NDB primary, DME auxiliary
//     8   ILS-DME                 → kOther         ILS not enroute-routable
//     9   DME (EXCLUDING ILS-DME) → kDme           standalone DME
//
// Types 6 and ≥10 absent from real data; fall through to kOther.
// Note: Type 9 DME idents are 1-3 chars (some NDB-like: 'AS','JA','MS').
// These are standalone DMEs (UHF-paired), not NDBs — kDme is correct.
WaypointKind NavaidKindFromType(int type_id) {
  switch (type_id) {
    case 1:  // VOR
    case 2:  // VORTAC
    case 4:  // VOR-DME
      return WaypointKind::kVor;
    case 3:  // TACAN
    case 9:  // DME (excluding ILS-DME)
      return WaypointKind::kDme;
    case 5:  // NDB
    case 7:  // NDB-DME
      return WaypointKind::kNdb;
    case 8:  // ILS-DME
    default:
      return WaypointKind::kOther;
  }
}

// Fenix stores navaid frequencies as 8-digit packed BCD -- one decimal digit per
// hex nibble, encoding freq*10000 (the ARINC 424 BCD frequency convention). E.g.
// LAX VORTAC 113.6 MHz is stored as 0x01136000 (nibbles "01136000" -> 1136000 ->
// /10000 = 113.6); an NDB at 292.0 kHz is 0x02920000. sqlite3_column_int returns
// the raw packed integer, so decode it here, then scale to the
// NavaidDetail.freq_raw contract (kHz for NDBs, MHz*100 for VOR/DME/ILS) to match
// DFD1/DFD2. A nibble >= 10 means the value is not BCD (never seen in real
// data); return -1 so the caller stores 0 instead of decoding garbage.
double DecodeFenixBcdFreq(int packed) {
  double value = 0.0;
  for (int shift = 28; shift >= 0; shift -= 4) {
    const int digit = (packed >> shift) & 0xF;
    if (digit > 9) {
      return -1.0;
    }
    value = value * 10.0 + static_cast<double>(digit);
  }
  return value / 10000.0;
}

// ---- altitude constraint parsing ----------------------------------------

// Parse a Fenix Alt text into an AltitudeConstraint.  The Fenix format is a
// compact concatenation of ARINC 424 altitude suffixes:
//   "<num>A" or "<num>+" → at or above
//   "<num>-"             → at or below
//   "<num>B"             → at or below (single B with no second part)
//   "<upper>B<lower>A"   → between: upper bound from B, lower bound from A
//   "<num>"  (bare)      → at
// ARINC: B = at-or-below, A = at-or-above.  Fenix joins the two into one
// string for between constraints.  AltitudeConstraint stores alt1_ft = upper
// bound, alt2_ft = lower bound (ARINC convention).
AltitudeConstraint ParseFenixAlt(const std::string& alt_text) {
  if (alt_text.empty()) {
    return {};
  }

  // Parse the first <num> segment.
  size_t pos = 0;
  while (pos < alt_text.size() && alt_text[pos] >= '0' && alt_text[pos] <= '9') {
    ++pos;
  }
  if (pos == 0) {
    return {};
  }

  int val1 = 0;
  auto [ptr1, ec1] = std::from_chars(alt_text.data(), alt_text.data() + pos, val1);
  if (ec1 != std::errc{}) {
    return {};
  }

  char desc1 = (pos < alt_text.size()) ? alt_text[pos] : '\0';
  ++pos;  // advance past the first suffix

  // If the first suffix is B, look for a second <num><suffix> pair.
  if (desc1 == 'B' && pos < alt_text.size()) {
    size_t num2_start = pos;
    while (pos < alt_text.size() && alt_text[pos] >= '0' && alt_text[pos] <= '9') {
      ++pos;
    }
    if (pos > num2_start) {
      int val2 = 0;
      auto [ptr2, ec2] = std::from_chars(alt_text.data() + num2_start, alt_text.data() + pos, val2);
      char desc2 = (pos < alt_text.size()) ? alt_text[pos] : '\0';
      if (ec2 == std::errc{} && (desc2 == 'A' || desc2 == '+')) {
        // "<upper>B<lower>A" → between
        return AltitudeConstraint{AltConstraintKind::kBetween, val1, val2};
      }
    }
  }

  // Single-segment format.
  AltConstraintKind kind = AltConstraintKind::kAt;
  if (desc1 == '+' || desc1 == 'A') {
    kind = AltConstraintKind::kAtOrAbove;
  } else if (desc1 == '-' || desc1 == 'B') {
    kind = AltConstraintKind::kAtOrBelow;
  }
  return AltitudeConstraint{kind, val1, 0};
}

// ---- cycle extraction ---------------------------------------------------

uint32_t ParseFenixCycle(sqlite3* conn) {
  Result<SqliteStmt> stmt =
      Prepare(conn, std::format("SELECT val FROM {} WHERE key='CycleName'", kTblConfig));
  if (!stmt) {
    return 0;
  }
  Result<bool> row = Step(stmt.value().get());
  if (!row || !row.value()) {
    return 0;
  }
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
    Result<SqliteStmt> s =
        Prepare(conn, std::format("SELECT ID, Country FROM {}", kTblWaypointLookup));
    if (!s) {
      return Result<void>::Err(s.error());
    }
    sqlite3_stmt* stmt = s.value().get();
    Result<void> rows =
        ForEachRow(stmt, [&] { region_by_id[ColumnInt(stmt, 0)] = ColumnText(stmt, 1); });
    if (!rows) {
      return Result<void>::Err(rows.error());
    }
  }

  // Build navaid kind lookup: NavaidID → Type.
  std::unordered_map<int, int> navaid_type_of;
  {
    Result<SqliteStmt> s = Prepare(conn, std::format("SELECT ID, Type FROM {}", kTblNavaids));
    if (!s) {
      return Result<void>::Err(s.error());
    }
    sqlite3_stmt* stmt = s.value().get();
    Result<void> rows =
        ForEachRow(stmt, [&] { navaid_type_of[ColumnInt(stmt, 0)] = ColumnInt(stmt, 1); });
    if (!rows) {
      return Result<void>::Err(rows.error());
    }
  }

  // 0=ID 1=Ident 2=Latitude 3=Longtitude 4=NavaidID
  Result<SqliteStmt> s = Prepare(
      conn, std::format("SELECT w.ID, w.Ident, w.Latitude, w.Longtitude, w.NavaidID FROM {} w",
                        kTblWaypoints));
  if (!s) {
    return Result<void>::Err(s.error());
  }

  sqlite3_stmt* stmt = s.value().get();
  int skipped_long = 0;
  Result<void> rows = ForEachRow(stmt, [&] {
    int id = ColumnInt(stmt, 0);
    std::string ident_str = ColumnText(stmt, 1);
    double lat = ColumnDouble(stmt, 2);
    double lon = ColumnDouble(stmt, 3);
    int nav_id = ColumnOptInt(stmt, 4);

    if (ident_str.size() > FixedIdent::kIdentCap) {
      BF_LOG_WARN("fenix: skipping waypoint '{}' (ident too long for FixedIdent, {} > {})",
                  ident_str, ident_str.size(), FixedIdent::kIdentCap);
      ++skipped_long;
      return;
    }

    std::string icao_code;
    auto rit = region_by_id.find(id);
    if (rit != region_by_id.end()) {
      icao_code = rit->second;
    }

    WaypointKind kind = WaypointKind::kFix;
    if (nav_id > 0) {
      auto nit = navaid_type_of.find(nav_id);
      if (nit != navaid_type_of.end()) {
        kind = NavaidKindFromType(nit->second);
      }
    }

    data.waypoints.push_back(Waypoint{Ident{ident_str, icao_code}, Coordinate{lat, lon}, kind});
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  if (skipped_long > 0) {
    BF_LOG_WARN("fenix: {} waypoint(s) skipped (ident too long for FixedIdent)", skipped_long);
  }
  return Result<void>::Ok();
}

// ---- navaid details -----------------------------------------------------

Result<void> LoadNavaidDetails(sqlite3* conn, NavData& data) {
  // Navigraph NavaidDetail, persisted for display and lookup-only queries.
  // 0=Ident 1=Type 2=Elevation 3=Freq 4=Range
  Result<SqliteStmt> s =
      Prepare(conn, std::format("SELECT Ident, Type, Elevation, Freq, Range FROM {}", kTblNavaids));
  if (!s) {
    return Result<void>::Err(s.error());
  }

  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&] {
    NavaidDetail detail;
    detail.ident = Ident{ColumnText(stmt, 0), ""};
    detail.kind = NavaidKindFromType(ColumnInt(stmt, 1));
    detail.elev_ft = ColumnInt(stmt, 2);
    // Fenix Freq is packed BCD of freq*10000; decode to the natural unit (MHz for
    // VOR/DME/ILS, kHz for NDB) then scale to the freq_raw contract (kHz as-is
    // for NDBs, MHz*100 otherwise), matching DFD1/DFD2. Invalid BCD -> 0.
    const double freq_value = DecodeFenixBcdFreq(ColumnInt(stmt, 3));
    if (freq_value < 0.0) {
      detail.freq_raw = 0;
    } else if (detail.kind == WaypointKind::kNdb) {
      detail.freq_raw = static_cast<int>(std::lround(freq_value));
    } else {
      detail.freq_raw = static_cast<int>(std::lround(freq_value * kCentiScale));
    }
    detail.range_nm = ColumnDouble(stmt, 4);
    data.navaid_details.push_back(detail);
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  return Result<void>::Ok();
}

// ---- airways ------------------------------------------------------------

Result<void> LoadAirways(sqlite3* conn, NavData& data) {
  // Build airway name lookup.
  std::unordered_map<int, std::string> airway_name;
  {
    Result<SqliteStmt> s = Prepare(conn, std::format("SELECT ID, Ident FROM {}", kTblAirways));
    if (!s) {
      return Result<void>::Err(s.error());
    }
    sqlite3_stmt* stmt = s.value().get();
    Result<void> rows =
        ForEachRow(stmt, [&] { airway_name[ColumnInt(stmt, 0)] = ColumnText(stmt, 1); });
    if (!rows) {
      return Result<void>::Err(rows.error());
    }
  }

  // Build waypoint ident/region lookup.
  std::unordered_map<int, Ident> wp_by_id;
  {
    Result<SqliteStmt> s = Prepare(
        conn,
        std::format(
            "SELECT w.ID, w.Ident, COALESCE(l.Country,'') FROM {} w LEFT JOIN {} l ON w.ID = l.ID",
            kTblWaypoints, kTblWaypointLookup));
    if (!s) {
      return Result<void>::Err(s.error());
    }
    sqlite3_stmt* stmt = s.value().get();
    Result<void> rows = ForEachRow(stmt, [&] {
      wp_by_id[ColumnInt(stmt, 0)] = Ident{ColumnText(stmt, 1), ColumnText(stmt, 2)};
    });
    if (!rows) {
      return Result<void>::Err(rows.error());
    }
  }

  // 0=AirwayID 1=Level 2=Waypoint1ID 3=Waypoint2ID 4=IsStart 5=IsEnd
  Result<SqliteStmt> s =
      Prepare(conn, std::format("SELECT al.AirwayID, al.Level, al.Waypoint1ID, al.Waypoint2ID, "
                                "al.IsStart, al.IsEnd FROM {} al",
                                kTblAirwayLegs));
  if (!s) {
    return Result<void>::Err(s.error());
  }

  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&] {
    int aid = ColumnInt(stmt, 0);
    std::string level = ColumnText(stmt, 1);
    int wp1 = ColumnInt(stmt, 2);
    int wp2 = ColumnInt(stmt, 3);

    auto nm = airway_name.find(aid);
    auto fm = wp_by_id.find(wp1);
    auto to = wp_by_id.find(wp2);
    if (nm == airway_name.end() || fm == wp_by_id.end() || to == wp_by_id.end()) {
      return;
    }

    AirwayConnection conn;
    conn.from = fm->second;
    conn.to = to->second;
    conn.segment.name = nm->second;
    conn.segment.level = ParseAirwayLevel(level);
    // Fenix schema does not encode per-segment direction restrictions;
    // default to bidirectional as in earth_awy.dat rows without 'F'/'B'.
    conn.segment.direction = AirwayDirection::kBoth;
    data.airways.push_back(conn);
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  return Result<void>::Ok();
}

// ---- airports -----------------------------------------------------------

Result<void> LoadAirports(sqlite3* conn, NavData& data) {
  // Build an ICAO → region mapping from WaypointLookup.Country.  Each airport
  // inherits the Country of the waypoints its terminal procedures reference,
  // keeping the region system consistent between airports and waypoints.
  // Airports without procedures fall back to the empty region.
  std::unordered_map<std::string, std::string> icao_region;
  {
    Result<SqliteStmt> ms = Prepare(
        conn,
        std::format("SELECT DISTINCT a.ICAO, wl.Country FROM {} a JOIN ", kTblAirports) +
            std::format("{} t ON t.AirportID = a.ID JOIN ", kTblTerminals) +
            std::format("{} tl ON tl.TerminalID = t.ID JOIN ", kTblTerminalLegs) +
            std::format("{} wl ON wl.ID = tl.WptID WHERE wl.Country != ''", kTblWaypointLookup));
    if (ms) {
      sqlite3_stmt* mstmt = ms.value().get();
      Result<void> rows = ForEachRow(
          mstmt, [&] { icao_region.try_emplace(ColumnText(mstmt, 0), ColumnText(mstmt, 1)); });
      if (!rows) {
        return Result<void>::Err(rows.error());
      }
    }
  }

  // 0=ICAO 1=Latitude 2=Longtitude 3=Elevation
  Result<SqliteStmt> s = Prepare(
      conn, std::format("SELECT ICAO, Latitude, Longtitude, Elevation FROM {}", kTblAirports));
  if (!s) {
    return Result<void>::Err(s.error());
  }

  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&] {
    std::string icao = ColumnText(stmt, 0);
    auto rit = icao_region.find(icao);
    std::string icao_code = (rit != icao_region.end()) ? rit->second : "";
    data.airports.push_back(Airport{icao, icao_code,
                                    Coordinate{ColumnDouble(stmt, 1), ColumnDouble(stmt, 2)},
                                    ColumnInt(stmt, 3)});
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  return Result<void>::Ok();
}

// ---- holdings -----------------------------------------------------------

Result<void> LoadHoldings(sqlite3* conn, NavData& data) {
  // 0=waypoint_identifier 1=region_code 2=icao_code
  // 3=inbound_holding_course 4=leg_time 5=leg_length 6=turn_direction
  // 7=minimum_altitude 8=maximum_altitude 9=holding_speed
  Result<SqliteStmt> s = Prepare(
      conn,
      std::format(
          "SELECT waypoint_identifier, region_code, icao_code, inbound_holding_course, leg_time, "
          "leg_length, turn_direction, minimum_altitude, maximum_altitude, holding_speed FROM {}",
          kTblHoldings));
  if (!s) {
    return Result<void>::Err(s.error());
  }

  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&] {
    HoldFix h;
    // col1=region_code is the airport id (or "ENRT"), NOT a region; col2=icao_code
    // is the 2-char ICAO region. Mirror dfd1/dfd2 LoadHoldings (dfd-loader-data-traps).
    h.fix = Ident{ColumnText(stmt, 0), ColumnText(stmt, 2)};  // fix.arinc424_icao_code <- icao_code
    h.airport_icao = ColumnText(stmt, 1);                     // <- region_code (airport/"ENRT")
    if (h.airport_icao.empty()) {
      h.airport_icao = "ENRT";
    }
    h.inbound_course = ColumnDouble(stmt, 3);
    h.leg_time_min = ColumnOptDouble(stmt, 4);
    h.leg_dist_nm = ColumnOptDouble(stmt, 5);
    h.turn_dir = EqualsCode(ColumnText(stmt, 6), kTurnLeft) ? kTurnLeft : kTurnRight;

    int mn = ColumnOptInt(stmt, 7);
    int mx = ColumnOptInt(stmt, 8);
    int spd = ColumnOptInt(stmt, 9);
    h.min_alt_ft = mn > 0 ? mn : 0;
    h.max_alt_ft = (mx > 0 && mx < kUnknownAltitudeFt) ? mx : 0;
    h.speed_limit_kt = spd > 0 ? spd : 0;
    data.hold_fixes.push_back(h);
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
  }
  return Result<void>::Ok();
}

// ---- MORA grid ----------------------------------------------------------

Result<void> LoadMoraGrid(sqlite3* conn, NavData& data) {
  // Fenix GridMora: one row per 1° lat × 30 columns (mora01..mora30).
  // Values are hundreds of feet as text (e.g. "010" = 1000 ft).
  // 0=starting_latitude 1=starting_longitude 2..31=mora01..mora30
  Result<SqliteStmt> s = Prepare(
      conn, std::format("SELECT starting_latitude, starting_longitude, "
                        "mora01,mora02,mora03,mora04,mora05,mora06,mora07,mora08,mora09,mora10,"
                        "mora11,mora12,mora13,mora14,mora15,mora16,mora17,mora18,mora19,mora20,"
                        "mora21,mora22,mora23,mora24,mora25,mora26,mora27,mora28,mora29,mora30 "
                        "FROM {} ORDER BY starting_latitude, starting_longitude",
                        kTblGridMora));
  if (!s) {
    return Result<void>::Err(s.error());
  }

  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&] {
    int start_lat = ColumnInt(stmt, 0);
    int start_lon = ColumnInt(stmt, 1);

    for (int col = 0; col < 30; ++col) {
      std::string txt = ColumnText(stmt, 2 + col);
      if (txt.empty()) {
        continue;
      }
      int mora_val = 0;
      auto [ptr, ec] = std::from_chars(txt.data(), txt.data() + txt.size(), mora_val);
      if (ec != std::errc{} || mora_val == 0) {
        continue;
      }

      // MORA cells are flight levels (hundreds of feet) keyed by signed
      // geographic degrees (lat in [-90,89], lon in [-180,179]); SetCell /
      // Index reject anything outside that range. The DB text already encodes
      // the flight level ("010" = FL10 = 1000 ft), so pass start_lat and the
      // signed column longitude straight through -- no +180/+360 remap (which
      // pushed the entire south/west hemisphere out of range and silently
      // dropped it) and no *100 amplification (which made every value 100x too
      // large and overflowed int16_t on high-terrain cells). This matches the
      // dfd1 / xplane12 loaders.
      data.mora.SetCell(start_lat, start_lon + col, static_cast<int16_t>(mora_val));
    }
  });
  if (!rows) {
    return Result<void>::Err(rows.error());
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

// ---- shared terminal-procedure core (single scan) -----------------------
//
// Both the bulk LoadProcedures path and the single-airport LoadProcedure path
// build procedures from the same Terminals + TerminalLegs + Runways tables.
// To keep the two paths from diverging (and to preserve the dfd1-style single
// TerminalLegs scan for cache-build performance), they share one core:
// BuildAirportProcedures does a single Terminals scan, a single TerminalLegs
// scan, and a single Runways scan, then emits one CifpData per airport via the
// shared EmitTerminalProcedure.

// One procedure's legs grouped by transition.  `runway` holds the runway ident
// only when `transition` is a runway transition (RW-prefixed, e.g. "RW18L");
// otherwise it is empty (enroute/IAF transitions name a fix, not a runway).
// "ALL" legs are split out into common_legs below. `route_type` is taken from
// TerminalLegs.Type (ARINC route type for approaches; numeric for SID/STAR).
struct LegGroup {
  std::vector<ProcedureLeg> legs;
  std::string transition;
  std::string runway;
  int route_type = 0;
};

// One TerminalLegs scan → leg_groups (keyed by TerminalID) + common_legs
// (the "ALL" transition legs, appended to every per-runway procedure).
// When `allowed_tids` is non-null, only legs whose TerminalID is in the set
// are processed (the single-airport path); when null, every leg is processed
// (the bulk path).  A single scan either way — no per-airport re-scan.
Result<void> BuildLegGroups(sqlite3* conn, const std::unordered_set<int>* allowed_tids,
                            std::unordered_map<int, std::vector<LegGroup>>& leg_groups,
                            std::unordered_map<int, std::vector<ProcedureLeg>>& common_legs) {
  // Waypoint ident/region lookup for leg-fix resolution.
  std::unordered_map<int, std::pair<std::string, std::string>> wp_info;
  {
    Result<SqliteStmt> ws = Prepare(
        conn,
        std::format(
            "SELECT w.ID, w.Ident, COALESCE(l.Country,'') FROM {} w LEFT JOIN {} l ON w.ID = l.ID",
            kTblWaypoints, kTblWaypointLookup));
    if (!ws) {
      return Result<void>::Err(ws.error());
    }
    sqlite3_stmt* stmt = ws.value().get();
    Result<void> rows = ForEachRow(
        stmt, [&]() { wp_info[ColumnInt(stmt, 0)] = {ColumnText(stmt, 1), ColumnText(stmt, 2)}; });
    if (!rows) {
      return Result<void>::Err(rows.error());
    }
  }

  // Scan the whole TerminalLegs table once; group by TerminalID + transition.
  {
    Result<SqliteStmt> ls = Prepare(
        conn,
        std::format(
            "SELECT tl.TerminalID, tl.Transition, tl.TrackCode, tl.Course, tl.Distance, tl.Alt, "
            "tl.TurnDir, tl.WptID, ex.SpeedLimit, tl.WptDescCode, tl.Type FROM {} tl LEFT JOIN ",
            kTblTerminalLegs) +
            std::format("{}Ex ex ON tl.ID = ex.ID ORDER BY tl.TerminalID, tl.ID",
                        kTblTerminalLegs));
    if (!ls) {
      return Result<void>::Err(ls.error());
    }
    sqlite3_stmt* stmt = ls.value().get();
    Result<void> rows = ForEachRow(stmt, [&]() {
      int tid = ColumnInt(stmt, 0);
      if (allowed_tids && allowed_tids->find(tid) == allowed_tids->end()) {
        return;
      }

      std::string trans = ColumnText(stmt, 1);
      ProcedureLeg leg;
      leg.path_term = ParsePathTerminator(ColumnText(stmt, 2));
      // Unknown TrackCode → skip the entire leg.  Real data has 'RWYTF'×4 and
      // 'RWY15'×2 — these legs are discarded silently.
      if (leg.path_term == PathTerminator::kUnknown) {
        return;
      }

      int wpt_id = ColumnOptInt(stmt, 7);
      if (wpt_id > 0) {
        auto wit = wp_info.find(wpt_id);
        if (wit != wp_info.end()) {
          const std::string& ident = wit->second.first;
          const std::string& icao_code = wit->second.second;
          Result<FixedIdent> fixed = FixedIdent::FromParts(ident, icao_code);
          if (!fixed) {
            BF_LOG_WARN("fenix: skipping procedure leg with ident '{}': {}", ident,
                        fixed.error().message);
            return;
          }
          leg.fix = std::move(fixed).value();
        }
      }

      leg.course_deg = static_cast<float>(ColumnDouble(stmt, 3));
      leg.distance_nm = static_cast<float>(ColumnOptDouble(stmt, 4));
      leg.set_alt(ParseFenixAlt(ColumnText(stmt, 5)));
      std::string td = ColumnText(stmt, 6);
      // TurnDir: 'L'/'R' only; anything else ('E' ×114 in real data) becomes
      // '\0'.  'E' appears to mean "either" — '\0' fallback is acceptable.
      leg.turn_dir = EqualsCode(td, kTurnLeft)    ? kTurnLeft
                     : EqualsCode(td, kTurnRight) ? kTurnRight
                                                  : '\0';
      double spd = ColumnOptDouble(stmt, 8);
      // uint16_t: clamp to [0, 65535] so a corrupt out-of-range speed does not
      // wrap (matches dfd1/dfd2). Real speeds are well below this.
      leg.speed_limit_kt = static_cast<uint16_t>(std::clamp<double>(
          spd > 0.0 ? spd : 0.0, 0.0, static_cast<double>(std::numeric_limits<uint16_t>::max())));
      // NOTE: Fenix schema has no RNP column in TerminalLegs/TerminalLegsEx,
      // so ProcedureLeg.rnp_centinm stays 0 (dfd1/dfd2/xplane12 load it).
      leg.is_mapt = IsMaptDesc(ColumnText(stmt, 9));
      const int route_type = ParseRouteTypeToken(ColumnText(stmt, 10));

      auto& groups = leg_groups[tid];
      if (groups.empty() || groups.back().transition != trans) {
        // `runway` is set only when this transition is a runway transition
        // (ident prefixed "RW", e.g. "RW18L"); enroute/IAF transitions carry a
        // fix name, not a runway, so they leave `runway` empty. Mirrors the
        // dfd1/dfd2/cifp loaders' RW-prefix guard. Approach runway comes from
        // Terminals.Rwy at emit time (not from Transition).
        groups.push_back(
            LegGroup{{}, trans, trans.rfind(kRunwayPrefix, 0) == 0 ? trans : "", route_type});
      }
      groups.back().legs.push_back(std::move(leg));
    });
    if (!rows) {
      return Result<void>::Err(rows.error());
    }
  }

  // Split "ALL" transitions into common_legs; every per-runway procedure
  // appends them.
  for (auto& [tid, groups] : leg_groups) {
    for (auto it = groups.begin(); it != groups.end();) {
      if (it->transition == "ALL") {
        common_legs[tid] = std::move(it->legs);
        it = groups.erase(it);
      } else {
        ++it;
      }
    }
  }

  return Result<void>::Ok();
}

// Emit one terminal's procedures (common segment + per-runway transitions)
// into `cifp`.  Shared by the bulk and single-airport paths so both produce
// byte-identical procedure structures.  `leg_groups` is read (not moved) so
// the bulk path can reuse it across airports.
void EmitTerminalProcedure(int tid, const std::unordered_map<int, Procedure>& proc_by_tid,
                           const std::unordered_map<int, std::vector<LegGroup>>& leg_groups,
                           const std::unordered_map<int, std::vector<ProcedureLeg>>& common_legs,
                           CifpData& cifp) {
  auto pit = proc_by_tid.find(tid);
  if (pit == proc_by_tid.end()) {
    return;
  }
  auto git = leg_groups.find(tid);
  if (git == leg_groups.end()) {
    return;
  }
  const Procedure& base = pit->second;
  auto cit = common_legs.find(tid);

  // Common-segment procedure (transition="", no runway) — the most direct
  // path to the exit fix.
  if (cit != common_legs.end() && !cit->second.empty()) {
    Procedure proc = base;
    proc.transition_ident = "";
    // Approaches: Terminals.Rwy is the structured runway (already normalized
    // into base.runway). SID/STAR common segments stay runway-independent.
    if (base.type != ProcedureType::kApproach) {
      proc.runway = "";
    }
    proc.legs = cit->second;
    cifp.procedures.push_back(std::move(proc));
  }

  // Per-runway-transition procedures with common legs appended.
  for (const auto& group : git->second) {
    Procedure proc = base;
    proc.transition_ident = group.transition;
    proc.route_type = group.route_type;
    if (base.type == ProcedureType::kApproach) {
      // Keep Terminals.Rwy (normalized into base.runway); Transition is an IAF.
    } else {
      proc.runway = group.runway;
    }
    proc.legs = group.legs;
    if (cit != common_legs.end()) {
      proc.legs.insert(proc.legs.end(), cit->second.begin(), cit->second.end());
    }
    cifp.procedures.push_back(std::move(proc));
  }
}

// Build CifpData (procedures + runways) for each airport in `airport_ids`.
// When `airport_ids` is null, every airport is processed (bulk path).  One
// Terminals scan, one TerminalLegs scan, one Runways scan — the bulk path
// scans the whole table, while the single-airport path narrows the Terminals
// and Runways scans with an indexed `WHERE AirportID = ?` instead of a full
// table scan plus an in-code filter.  Returns airport_id → CifpData, omitting
// airports with neither procedures nor runways.
Result<std::unordered_map<int, CifpData>> BuildAirportProcedures(
    sqlite3* conn, const std::unordered_set<int>* airport_ids) {
  // Terminal → (airport, procedure type/name) + airport → terminal list.
  std::unordered_map<int, Procedure> proc_by_tid;
  std::unordered_map<int, std::vector<int>> airport_tids;
  // Single-airport requests use an indexed WHERE; bulk scans all rows.  Either
  // way there is one Terminals scan — the single-airport path stays O(1) on the
  // index instead of degrading to a full table scan plus an in-code filter.
  const bool single_airport = airport_ids != nullptr && airport_ids->size() == 1;
  {
    std::string sql =
        single_airport
            ? std::format(
                  "SELECT ID, AirportID, Proc, Name, Rwy FROM {} WHERE AirportID = ? ORDER BY ID",
                  kTblTerminals)
            : std::format("SELECT ID, AirportID, Proc, Name, Rwy FROM {} ORDER BY AirportID, ID",
                          kTblTerminals);
    Result<SqliteStmt> ts = Prepare(conn, sql);
    if (!ts) {
      return Result<std::unordered_map<int, CifpData>>::Err(ts.error());
    }
    sqlite3_stmt* stmt = ts.value().get();
    if (single_airport) {
      sqlite3_bind_int(stmt, 1, *airport_ids->begin());
    }
    Result<void> rows = ForEachRow(stmt, [&]() {
      int tid = ColumnInt(stmt, 0);
      int aid = ColumnInt(stmt, 1);
      // In-code filter only applies to the bulk scan; the single-airport path
      // is already narrowed by the WHERE above.
      if (airport_ids && !single_airport && airport_ids->find(aid) == airport_ids->end()) {
        return;
      }
      // Fenix Proc: '2'=SID, '1'=STAR, everything else (e.g. '3' RNAV
      // approach, ~34k rows) → kApproach silently.
      std::string pt = ColumnText(stmt, 2);
      Procedure proc;
      if (pt == "2") {
        proc.type = ProcedureType::kSid;
      } else if (pt == "1") {
        proc.type = ProcedureType::kStar;
      } else {
        proc.type = ProcedureType::kApproach;
      }
      proc.name = ColumnText(stmt, 3);
      // Approaches carry a structured Rwy column ("18"/"06L"); normalize to the
      // internal RW-prefixed form. SID/STAR runway still comes from an
      // RW-prefixed Transition at emit time.
      if (proc.type == ProcedureType::kApproach) {
        proc.runway = NormalizeRunwayIdent(ColumnText(stmt, 4));
      }
      proc_by_tid[tid] = std::move(proc);
      airport_tids[aid].push_back(tid);
    });
    if (!rows) {
      return Result<std::unordered_map<int, CifpData>>::Err(rows.error());
    }
  }

  // BuildLegGroups filters the TerminalLegs scan by TerminalID, so narrow the
  // request to the TerminalIDs that belong to the requested airport(s).
  std::unordered_set<int> allowed_tids;
  if (airport_ids) {
    for (auto& [aid, tids] : airport_tids) {
      for (int tid : tids) {
        allowed_tids.insert(tid);
      }
    }
  }
  std::unordered_map<int, std::vector<LegGroup>> leg_groups;
  std::unordered_map<int, std::vector<ProcedureLeg>> common_legs;
  Result<void> lr =
      BuildLegGroups(conn, airport_ids ? &allowed_tids : nullptr, leg_groups, common_legs);
  if (!lr) {
    return Result<std::unordered_map<int, CifpData>>::Err(lr.error());
  }

  // Runways by airport.  The single-airport query also uses the indexed WHERE
  // (mirrors the Terminals path above); bulk scans all rows.
  std::unordered_map<int, std::vector<Runway>> runways_by_airport;
  {
    std::string rsql = single_airport ? std::format(
                                            "SELECT AirportID, Ident, Latitude, Longtitude, "
                                            "Elevation FROM {} WHERE AirportID = ? ORDER BY Ident",
                                            kTblRunways)
                                      : std::format(
                                            "SELECT AirportID, Ident, Latitude, Longtitude, "
                                            "Elevation FROM {} ORDER BY AirportID",
                                            kTblRunways);
    Result<SqliteStmt> rs = Prepare(conn, rsql);
    if (!rs) {
      return Result<std::unordered_map<int, CifpData>>::Err(rs.error());
    }
    sqlite3_stmt* stmt = rs.value().get();
    if (single_airport) {
      sqlite3_bind_int(stmt, 1, *airport_ids->begin());
    }
    Result<void> rows = ForEachRow(stmt, [&]() {
      int aid = ColumnInt(stmt, 0);
      if (airport_ids && !single_airport && airport_ids->find(aid) == airport_ids->end()) {
        return;
      }
      runways_by_airport[aid].push_back(
          Runway{std::format("{}{}", kRunwayPrefix, ColumnText(stmt, 1)),
                 Coordinate{ColumnDouble(stmt, 2), ColumnDouble(stmt, 3)}, ColumnInt(stmt, 4)});
    });
    if (!rows) {
      return Result<std::unordered_map<int, CifpData>>::Err(rows.error());
    }
  }

  std::unordered_map<int, CifpData> result;
  // Airports with procedures (and possibly runways).
  for (auto& [aid, tids] : airport_tids) {
    CifpData cifp;
    for (int tid : tids) {
      EmitTerminalProcedure(tid, proc_by_tid, leg_groups, common_legs, cifp);
    }
    auto rit = runways_by_airport.find(aid);
    if (rit != runways_by_airport.end()) {
      cifp.runways = std::move(rit->second);
    }
    if (!cifp.procedures.empty() || !cifp.runways.empty()) {
      result[aid] = std::move(cifp);
    }
  }
  // Airports with runways but no procedures.
  for (auto& [aid, rwys] : runways_by_airport) {
    if (result.find(aid) == result.end()) {
      CifpData cifp;
      cifp.runways = std::move(rwys);
      result[aid] = std::move(cifp);
    }
  }
  return Result<std::unordered_map<int, CifpData>>::Ok(std::move(result));
}

// ---- airport ICAO lookup -------------------------------------------------

Result<std::unordered_map<int, std::string>> LoadAirportIcaoMap(sqlite3* conn) {
  std::unordered_map<int, std::string> icao_of;
  Result<SqliteStmt> s = Prepare(conn, std::format("SELECT ID, ICAO FROM {}", kTblAirports));
  if (!s) {
    return Result<std::unordered_map<int, std::string>>::Err(s.error());
  }
  sqlite3_stmt* stmt = s.value().get();
  Result<void> rows = ForEachRow(stmt, [&] { icao_of[ColumnInt(stmt, 0)] = ColumnText(stmt, 1); });
  if (!rows) {
    return Result<std::unordered_map<int, std::string>>::Err(rows.error());
  }
  return Result<std::unordered_map<int, std::string>>::Ok(std::move(icao_of));
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
  if (!r) {
    return Result<NavData>::Err(r.error());
  }

  r = LoadNavaidDetails(conn, data);
  if (!r) {
    return Result<NavData>::Err(r.error());
  }

  r = LoadAirways(conn, data);
  if (!r) {
    return Result<NavData>::Err(r.error());
  }

  r = LoadAirports(conn, data);
  if (!r) {
    return Result<NavData>::Err(r.error());
  }

  r = LoadHoldings(conn, data);
  if (!r) {
    return Result<NavData>::Err(r.error());
  }

  r = LoadMoraGrid(conn, data);
  if (!r) {
    return Result<NavData>::Err(r.error());
  }

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

  // Single-scan core builds every airport's CifpData (procedures + runways).
  Result<std::unordered_map<int, CifpData>> built = BuildAirportProcedures(conn, nullptr);
  if (!built) {
    return Result<std::vector<AirportProcedureData>>::Err(built.error());
  }

  // Map airport id → ICAO; airports missing an ICAO are skipped.  Emit in
  // ascending airport_id order so the output (and any .bfdb cache built from
  // it) is reproducible across compilers and standard libraries.
  auto icao_map = LoadAirportIcaoMap(conn);
  if (!icao_map) {
    return Result<std::vector<AirportProcedureData>>::Err(icao_map.error());
  }
  auto& icao_of = icao_map.value();
  std::vector<int> aids;
  aids.reserve(built.value().size());
  for (const auto& [aid, cifp] : built.value()) {
    aids.push_back(aid);
  }
  std::sort(aids.begin(), aids.end());
  std::vector<AirportProcedureData> out;
  out.reserve(aids.size());
  for (int aid : aids) {
    auto iit = icao_of.find(aid);
    if (iit == icao_of.end()) {
      continue;
    }
    auto cit = built.value().find(aid);
    out.emplace_back(iit->second, std::move(cit->second));
  }
  return Result<std::vector<AirportProcedureData>>::Ok(std::move(out));
}

// Load procedures for a single airport (LoadProcedure path).  Delegates to the
// shared BuildAirportProcedures core with a one-element airport set, so it
// produces byte-identical structures to the bulk LoadProcedures path.
std::optional<CifpData> LoadAirportProcedures(sqlite3* conn, const std::string& icao) {
  int airport_id = -1;
  {
    Result<SqliteStmt> s =
        Prepare(conn, std::format("SELECT ID FROM {} WHERE ICAO = ?", kTblAirports));
    if (!s) {
      return std::nullopt;
    }
    sqlite3_bind_text(s.value().get(), 1, icao.c_str(), -1, SQLITE_TRANSIENT);
    Result<bool> row = Step(s.value().get());
    if (row && row.value()) {
      airport_id = ColumnInt(s.value().get(), 0);
    }
  }
  if (airport_id < 0) {
    return std::nullopt;
  }

  std::unordered_set<int> ids{airport_id};
  Result<std::unordered_map<int, CifpData>> built = BuildAirportProcedures(conn, &ids);
  if (!built) {
    return std::nullopt;
  }
  auto it = built.value().find(airport_id);
  if (it == built.value().end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<CifpData> FenixLoader::LoadProcedure(const std::string& source_dir,
                                                   const std::string& icao) const {
  Result<std::string> db_path = FindFenixDb(source_dir);
  if (!db_path) {
    return std::nullopt;
  }

  Result<sqlite3*> conn_result = AcquireConn(kLoaderName, db_path.value());
  if (!conn_result) {
    return std::nullopt;
  }

  return LoadAirportProcedures(conn_result.value(), icao);
}

}  // namespace bf
