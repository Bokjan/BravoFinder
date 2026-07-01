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
  std::string ident;
  std::string region;  // two-letter ICAO region code
  Coordinate coord;
  WaypointKind kind = WaypointKind::kFix;
  bool on_network = false;  // participates in the enroute airway network
};

// An airport node.
struct AirportInfo {
  std::string icao;
  std::string region;
  Coordinate coord;
  int elevation_ft = 0;
  bool has_procedures = false;  // whether CIFP terminal procedures are available
};

// One named terminal procedure (a single transition of a SID/STAR/approach).
struct ProcedureSummary {
  ProcedureType type = ProcedureType::kSid;
  std::string name;        // e.g. "DEEZZ5"
  std::string transition;  // runway "RW31L", fix "CANDR", or "ALL"
  std::string runway;      // resolved runway ident if the transition is one
};

// All terminal procedures published for one airport.
struct AirportProcedures {
  std::string icao;
  std::vector<ProcedureSummary> procedures;
};

// One directed segment of an airway: a hop between two consecutive fixes.
struct AirwayLeg {
  std::string from;   // fix ident
  std::string to;     // fix ident
  double distance_nm = 0.0;
  bool high = false;  // Jet (high) airway segment; false = Victor (low)
  int base_fl = 0;    // lowest usable flight level (0 = no limit)
  int top_fl = 0;     // highest usable flight level (0 = no limit)
};

// An airway as its set of directed segments. Segments are not chained into a
// single linear order: real airways branch and reverse, so the honest
// representation is the segment list, which the caller can chain if needed.
struct AirwayInfo {
  std::string name;
  std::vector<AirwayLeg> segments;
};

}  // namespace bf
