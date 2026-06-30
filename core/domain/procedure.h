#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/domain/ident.h"

namespace bf {

// ARINC 424 path-and-termination code: how a procedure leg is flown and what
// ends it. X-Plane CIFP uses the full set; all fourteen are recognized here.
// The first four (TF/IF/DF/CF) terminate at a definite fix and dominate (~85%
// of legs); the rest fly a heading/arc/altitude/hold and have no fixed end
// point, so they are collapsed to an equivalent edge when wiring the graph.
enum class PathTerminator {
  kTF,       // Track to Fix
  kIF,       // Initial Fix
  kDF,       // Direct to Fix
  kCF,       // Course to Fix
  kCA,       // Course to Altitude
  kFM,       // From Fix to Manual termination
  kVA,       // Heading to Altitude
  kVM,       // Heading to Manual termination
  kVI,       // Heading to Intercept
  kVR,       // Heading to Radial
  kRF,       // Constant Radius to Fix
  kVD,       // Heading to DME distance
  kHM,       // Hold to Manual termination
  kHF,       // Hold to Fix
  kUnknown,  // unrecognized code (kept so parsing never silently drops a leg)
};

// True for the "fly to a definite fix" terminators (TF/IF/DF/CF). Their fix is
// a real navigation point that can be resolved to a graph vertex; the others
// terminate on a heading/altitude/arc/hold and must be estimated.
bool TerminatesAtFix(PathTerminator t);

// Parse a two-letter CIFP path-terminator token (e.g. "TF"). Returns kUnknown
// for anything unrecognized.
PathTerminator ParsePathTerminator(std::string_view token);

// Short token for a path terminator (e.g. "TF"), for display/round-tripping.
std::string PathTerminatorName(PathTerminator t);

// The kind of altitude restriction a leg carries, from the CIFP altitude
// descriptor column ('+', '-', '@'/blank, 'B').
enum class AltConstraintKind {
  kNone,       // no altitude restriction on this leg
  kAt,         // cross at altitude one ('@' or blank with an altitude present)
  kAtOrAbove,  // cross at or above altitude one ('+')
  kAtOrBelow,  // cross at or below altitude one ('-')
  kBetween,    // between altitude two (lower) and altitude one (upper) ('B')
};

// An altitude restriction on a leg. Altitudes are in feet MSL as stored in the
// CIFP. For kBetween, alt1_ft is the upper bound and alt2_ft the lower.
struct AltitudeConstraint {
  AltConstraintKind kind = AltConstraintKind::kNone;
  int alt1_ft = 0;
  int alt2_ft = 0;
};

// One leg of a procedure: the path terminator, its (possibly empty) fix, and
// the course/distance/altitude data parsed from the CIFP row. Legs that do not
// terminate at a fix leave `fix` empty and rely on course/distance.
struct ProcedureLeg {
  Ident fix;  // empty ident for heading/altitude/manual-termination legs
  PathTerminator path_term = PathTerminator::kUnknown;
  double course_deg = 0.0;   // magnetic course (CIFP column, 0 if absent)
  double distance_nm = 0.0;  // route/leg distance (CIFP column, 0 if absent)
  AltitudeConstraint alt;

  // Whether this leg ends at a resolvable navigation fix.
  bool fix_is_definite() const { return TerminatesAtFix(path_term) && !fix.ident.empty(); }
};

// Which kind of terminal procedure this is.
enum class ProcedureType {
  kSid,       // Standard Instrument Departure
  kStar,      // Standard Terminal Arrival Route
  kApproach,  // instrument approach (APPCH)
};

// A single procedure (one transition of a SID/STAR/approach): an ordered chain
// of legs sharing a name, transition, and route type. One named procedure (e.g.
// "DEEZZ5") is published as several Procedure records, one per transition
// (runway transition, common segment, enroute transition).
struct Procedure {
  ProcedureType type = ProcedureType::kSid;
  std::string name;              // procedure name, e.g. "DEEZZ5" (column 3)
  std::string transition_ident;  // runway "RW31L", fix "CANDR", or "ALL" (col 4)
  std::string runway;            // resolved runway ident if transition is one
  int route_type = 0;            // ARINC 424 route type (column 2), kept raw
  std::vector<ProcedureLeg> legs;
};

// A runway threshold, parsed from a CIFP "RWY:" record. Used as a graph vertex
// when wiring procedures so a route can begin/end at the actual runway.
struct Runway {
  std::string ident;     // e.g. "RW31L"
  Coordinate threshold;  // threshold position
  int elevation_ft = 0;
};

}  // namespace bf
