// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/domain/procedure.h"
#include "core/domain/waypoint.h"

namespace bf {

// Result types for the NavDatabase batch lookup API. These are plain data,
// decoupled from the internal graph representation, so library users can consume
// them directly. Every batch lookup returns a vector<optional<Info>> parallel to
// the input: nullopt marks an ident/name that was not found.

// A navigation point (enroute waypoint or radio navaid).
struct WaypointInfo {
  std::string ident{};
  std::string region{};  // two-letter ICAO region code
  Coordinate coord{};
  WaypointKind kind = WaypointKind::kFix;
  bool on_network = false;  // participates in the enroute airway network (inbound or outbound edge)
};

// An airport node.
struct AirportInfo {
  std::string icao{};
  std::string region{};
  Coordinate coord{};
  int elevation_ft = 0;
  bool has_procedures = false;  // whether CIFP terminal procedures are available
};

// One named terminal procedure (a single transition of a SID/STAR/approach).
struct ProcedureSummary {
  ProcedureType type = ProcedureType::kSid;
  std::string name{};        // e.g. "DEEZZ5"
  std::string transition{};  // runway "RW31L", fix "CANDR", or "ALL"
  std::string runway{};      // resolved runway ident if the transition is one
};

// All terminal procedures published for one airport.
struct AirportProcedures {
  std::string icao{};
  std::vector<ProcedureSummary> procedures{};
};

// One leg of a procedure, surfaced for per-leg detail queries. Carries the
// leg-level fields ProcedureSummary omits. Absent values use the empty/zero
// sentinel: an empty `fix` for heading/altitude/manual-termination legs, empty
// `alt`, rnp_nm 0, turn_dir '\0', speed_limit_kt 0.
struct ProcedureLegInfo {
  std::string fix{};        // terminating fix ident (empty for non-fix legs)
  std::string path_term{};  // ARINC token, e.g. "TF", "RF"
  double course_deg = 0.0;  // magnetic course
  double distance_nm = 0.0;
  std::string alt{};       // "@5000" / "+2500" / "-4000" / "3000-6000" / ""
  double rnp_nm = 0.0;     // required navigation performance, NM (0 = absent)
  char turn_dir = '\0';    // 'L' / 'R' / '\0'
  int speed_limit_kt = 0;  // 0 = none
};

// One transition of a named procedure, with its ordered legs.
struct ProcedureDetail {
  ProcedureType type = ProcedureType::kSid;
  std::string name{};
  std::string transition{};
  std::string runway{};
  std::vector<ProcedureLegInfo> legs{};
};

// The per-leg detail of one named procedure at an airport. A published name
// (e.g. "DEEZZ5") maps to several transitions (runway/common/enroute), so this
// holds every Procedure record whose name matches the request.
struct AirportProcedureDetail {
  std::string icao{};
  std::string procedure{};  // requested procedure name (as matched)
  std::vector<ProcedureDetail> transitions{};
};

// One directed segment of an airway: a hop between two consecutive fixes.
struct AirwayLeg {
  std::string from{};  // fix ident
  std::string to{};    // fix ident
  double distance_nm = 0.0;
  bool high = false;  // Jet (high) airway segment; false = Victor (low)
  int base_fl = 0;    // lowest usable flight level (0 = no limit)
  int top_fl = 0;     // highest usable flight level (0 = no limit)
};

// An airway as its set of directed segments. Segments are not chained into a
// single linear order: real airways branch and reverse, so the honest
// representation is the segment list, which the caller can chain if needed.
struct AirwayInfo {
  std::string name{};
  std::vector<AirwayLeg> segments{};
};

// Detailed attributes of a radio navaid (VOR/NDB/DME/ILS), supplementing the
// basic WaypointInfo from LookupWaypoints with fields not needed for routing.
struct NavaidDetailInfo {
  std::string ident{};
  std::string region{};
  WaypointKind kind = WaypointKind::kOther;
  int elev_ft = 0;
  int freq_raw = 0;  // raw dat value: NDB = kHz; VOR/ILS/DME = MHz * 100
  double range_nm = 0.0;
  double heading = 0.0;  // VOR: slaved variation; ILS/LOC: localizer bearing; DME: bias
};

// One holding pattern at a fix, from earth_hold.dat.
struct HoldInfo {
  std::string fix_ident{};
  std::string fix_region{};
  std::string airport_icao{};  // "ENRT" for enroute holds
  double inbound_course = 0.0;
  double leg_time_min = 0.0;
  double leg_dist_nm = 0.0;
  char turn_dir = 'R';  // 'R' or 'L'
  int min_alt_ft = 0;
  int max_alt_ft = 0;      // 0 = no upper limit
  int speed_limit_kt = 0;  // 0 = no limit
};

}  // namespace bf
