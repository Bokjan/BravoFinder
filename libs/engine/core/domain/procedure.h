// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "core/domain/coordinate.h"
#include "core/domain/fixed_ident.h"
#include "core/domain/ident.h"

namespace bf {

// ARINC 424 path-and-termination code: how a procedure leg is flown and what
// ends it. The full corpus (all 14838 cycle-2601 airports) uses 23 distinct
// codes; the first four (TF/IF/DF/CF) terminate at a definite fix and dominate
// (the large majority of legs), while the rest fly a heading/course/arc/distance/
// altitude/hold and have no fixed end point, so they are collapsed to an
// equivalent edge when wiring the graph.
enum class PathTerminator {
  kTF,       // Track to Fix
  kIF,       // Initial Fix
  kDF,       // Direct to Fix
  kCF,       // Course to Fix
  kAF,       // Arc to Fix (constant-DME arc)
  kRF,       // Constant Radius to Fix
  kCA,       // Course to Altitude
  kFA,       // Track from Fix to Altitude
  kVA,       // Heading to Altitude
  kHA,       // Hold to Altitude
  kCD,       // Course to DME distance
  kFD,       // Track from Fix to DME distance
  kVD,       // Heading to DME distance
  kCI,       // Course to Intercept
  kVI,       // Heading to Intercept
  kCR,       // Course to Radial
  kVR,       // Heading to Radial
  kFC,       // Track from Fix for a Distance
  kFM,       // From Fix to Manual termination
  kVM,       // Heading to Manual termination
  kPI,       // Procedure turn (to Intercept)
  kHM,       // Hold to Manual termination
  kHF,       // Hold to Fix
  kUnknown,  // unrecognized code (kept so parsing never silently drops a leg)
};

// True for the "fly to a definite fix" terminators (TF/IF/DF/CF/AF/RF/HF -- the
// ARINC 424 codes ending in "F", to Fix). Their fix is a real navigation point
// that can be resolved to a graph vertex; the others terminate on a
// heading/altitude/arc/manual/hold-to-non-fix and must be estimated.
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

// Parse the altitude restriction from the descriptor and the two altitude
// values. `desc` is '+', '-', '@'/blank, or 'B' (the CIFP/DFD altitude
// descriptor); alt1/alt2 are feet as stored. Shared by the X-Plane CIFP parser
// and the DFD SQLite loaders so the column-extraction differs but the
// descriptor logic does not.
AltitudeConstraint ParseAltConstraint(std::string_view desc, int alt1, int alt2);

// One leg of a procedure: the path terminator, its (possibly empty) fix, and
// the course/distance/altitude data parsed from the CIFP row. Legs that do not
// terminate at a fix leave `fix` empty and rely on course/distance.
struct ProcedureLeg {
  // Compact 12-byte fixed ident (vs 64B Ident): the leg array is by far the
  // largest CIFP structure (~765k legs), so this cuts eager-mode resident memory
  // by ~40 MB. Empty ident for heading/altitude/manual-termination legs.
  FixedIdent fix;
  PathTerminator path_term = PathTerminator::kUnknown;
  double course_deg = 0.0;   // magnetic course (CIFP column, 0 if absent)
  double distance_nm = 0.0;  // route/leg distance (CIFP column, 0 if absent)
  AltitudeConstraint alt;

  // Compact encodings kept small because the leg array is ~765k entries (see the
  // FixedIdent note above): a double/string per field would cost megabytes.
  uint16_t rnp_centinm = 0;     // required navigation performance, hundredths of a
                                // nautical mile (0.30 NM -> 30); 0 means absent.
  uint16_t speed_limit_kt = 0;  // published speed limit in knots; 0 means none.
  char turn_dir = '\0';         // 'L'/'R' turn direction, or '\0' when unspecified.

  // Whether this leg ends at a resolvable navigation fix.
  bool fix_is_definite() const { return TerminatesAtFix(path_term) && !fix.IdentView().empty(); }
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
  std::string name{};              // procedure name, e.g. "DEEZZ5" (column 3)
  std::string transition_ident{};  // runway "RW31L", fix "CANDR", or "ALL" (col 4)
  std::string runway{};            // resolved runway ident if transition is one
  int route_type = 0;              // ARINC 424 route type (column 2), kept raw
  std::vector<ProcedureLeg> legs;
};

// A runway threshold, parsed from a CIFP "RWY:" record. Used as a graph vertex
// when wiring procedures so a route can begin/end at the actual runway.
struct Runway {
  std::string ident{};   // e.g. "RW31L"
  Coordinate threshold;  // threshold position
  int elevation_ft = 0;
};

// The procedures and runways parsed from one airport's terminal-procedure data
// (X-Plane CIFP files or DFD SQLite procedure tables). Pure data, not specific
// to any source format, so it lives in core rather than under a single loader.
struct CifpData {
  std::vector<Procedure> procedures;
  std::vector<Runway> runways;
};

}  // namespace bf
