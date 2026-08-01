// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/result.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"

namespace bf {

// The container-level header of a unified `.bfdb`, readable without decoding any
// section body. Holds the AIRAC cycle and provenance -- the single home for this
// metadata, which no longer lives per section. Used to catalog a directory of
// caches (see bfdb_inventory.h). The filename is only a hint; this header is
// authoritative.
struct UnifiedHeader {
  uint32_t cycle = 0;           // AIRAC cycle, e.g. 2601; 0 means no provenance
  std::string program_version;  // bf version that built this cache
  std::string source_loader;    // loader that produced the data (a name from the loader registry)
  std::string data_dir;         // the data dir used at build time (route default)
};

// The decoded contents of a unified `.bfdb`: the graph snapshot, an optional
// CIFP archive (on-demand, holding a pread handle into the file), and an
// optional navaid-detail archive, plus the container header.
struct UnifiedData {
  UnifiedHeader header;
  GraphSnapshot graph;
  std::optional<CifpArchive> cifp;         // absent if the file has no CIFP section
  std::optional<NavDetailArchive> detail;  // absent if the file has no detail section
};

// The unified `.bfdb` container: one file holding a graph section, an optional
// CIFP section, and an optional navaid-detail section, all sharing one global
// string pool. This class owns the file, the header, the section table, the
// pool, and the single container-level format version; the three section codecs
// (GraphCodec / CifpCodec / NavDetailCodec) own only their struct<->bytes
// mapping.
//
// On-disk layout:
//   [file header]  magic "BFDB", format_version, section_count, cycle,
//                  program_version, source_loader, data_dir, pool_len
//   [section table]  section_count * (type U32, offset U64, length U64);
//                    offset/length == 0 means the section is absent
//   [global string pool]  pool_len bytes, shared by all sections
//   [graph section] [cifp section] [detail section]  in section-table order
//
// The pool precedes the section bodies so a reader has it in memory before
// decoding any section, and CIFP on-demand can skip straight to fetching
// segments after reading the (small) directory.
class UnifiedCache {
 public:
  // The single on-disk format version for the whole container. Bump whenever ANY
  // section's layout or the container layout changes; older files are then
  // rejected and the user re-runs `bf build`.
  //
  // v4: unified container (graph + cifp + detail in one file), one global string
  //     pool, CIFP segments with no per-segment pool, `build` dropped from
  //     provenance. Supersedes the separate graph v3 / cifp v1 / detail v1 caches.
  // v5: edge 'flags' bitfield (kEdgeHigh/kEdgeBoth) replaced by a single 'level'
  //     byte holding AirwayLevel. The on-disk values coincide (0/1/2), but the
  //     byte's contract changed from bit flags to an enum value, so the version
  //     is bumped to retire the bitfield interpretation.
  // v6: CIFP ProcedureLeg gained rnp_centinm (U16), turn_dir (U8), and
  //     speed_limit_kt (U16), appended to each leg record; the CIFP section grew
  //     5 bytes per leg, so older files must be rebuilt.
  // v7: the vertex-record flags byte gained bit1 = has_inbound (bit0, formerly
  //     on_network, is now has_outbound). Same byte width, but v6 files decode
  //     has_inbound as all-false, which silently strips STAR entry gates (fixes
  //     reachable only via a forward-only airway) off the network — so v6 is
  //     rejected and must be rebuilt rather than read with a wrong flag.
  // v8: no layout change. The DFD loaders used to read each airway record's
  //     direction/level/altitude off the wrong row, shifting every restriction
  //     one leg forward (e.g. a forward-only 'F' barred the bidirectional leg
  //     before it). v7 files built from DFD carry those wrong edge directions and
  //     would silently reject valid routes, so v7 is retired to force a rebuild.
  // v9: no layout change. The DFD loaders used to chain every same-route_identifier
  //     fix sequence into one airway, but that identifier is not unique per
  //     physical airway (e.g. "V105" spans disjoint US/China/India strings). v8
  //     files built from DFD carry thousands of cross-string phantom legs (e.g. a
  //     ~5400 nm FMG->PADNO edge) that let routes teleport across oceans, so v8 is
  //     retired to force a rebuild once the loaders break the chain at the ARINC
  //     424 End-of-Airway marker (waypoint_description_code column 2 == 'E').
  // v10: no layout change. The DFD loaders keyed terminal waypoints by region_code
  //      (the airport the fix belongs to, e.g. "01OH") instead of icao_code (the
  //      2-char ICAO region). v9 DFD files store those fixes under a wrong,
  //      truncated region -- unreachable by airways/procedures and wrong in
  //      lookups -- so v9 is retired to force a rebuild once the key is corrected.
  // v11: no layout change. The dfd1 procedure loader returned early from its row
  //      scan and never flushed the last airport of each SID/STAR/IAP table, so
  //      v10 dfd1 files silently omit those procedures while staying byte- and
  //      version-valid. v10 is retired to force a rebuild of the now-complete data.
  // v12: no layout change. The Fenix MORA loader remapped south/west-hemisphere
  //      coords with +180/+360 (dropping the whole southern/western grid, MORA 0)
  //      and amplified every value 100x (overflowing int16_t on high-terrain
  //      cells, silently disabling the terrain floor). v11 Fenix-sourced files
  //      carry that wrong MORA grid while staying byte- and version-valid, so v11
  //      is retired to force a rebuild with correct MORA data.
  // v13: no field-size change (turn_dir still one U8), but its value semantics
  //      changed. The nav_detail hold turn_dir was encoded as 1-bit ('L'->1,
  //      else->0) and decoded back to 'L'/'R', collapsing any non-L value to 'R'
  //      -- inconsistent with cifp_codec's lossless full-char ProcedureLeg
  //      turn_dir. v13 stores the raw char losslessly. v12 files store 0/1, which
  //      the new full-char decode would read as '\0'/'\x01' (wrong), so v12 is
  //      retired. (Holds are always 'L'/'R' in practice, so this is
  //      consistency-only, not a data fix; done because the bump is free
  //      pre-release.)
  // v14: no layout change. The Fenix holding loader swapped the Holdings table's
  //      region_code and icao_code columns: fix.arinc424_icao_code received region_code
  //      (the airport id, e.g. "RJTT"/"ENRT") and airport_icao received icao_code
  //      (the 2-char region, e.g. "RJ"). v13 Fenix-sourced files carry swapped
  //      (terminal) or ENRT-polluted (enroute) hold fix_arinc424_icao_code/airport_icao
  //      while staying byte- and version-valid, so v13 is retired to force a
  //      rebuild. (DFD1/DFD2 were already correct; X-Plane unaffected.)
  // v15: no layout change. The Fenix procedure loader copied the TerminalLegs
  //      Transition value into Procedure::runway unconditionally, so every
  //      enroute/IAF transition (a fix name like "CANDR"/"COVIR", not a runway)
  //      carried a bogus non-empty runway. Under a --rwy-dep/--rwy-arr filter
  //      RunwayMatches then wrongly excluded those transitions from the
  //      connection set, and approaches surfaced IAF names as their runway in
  //      display/JSON. v14 Fenix-sourced files carry those wrong runway values
  //      while staying byte- and version-valid, so v14 is retired to force a
  //      rebuild. (DFD1/DFD2/X-Plane already gate runway on the RW prefix.)
  static constexpr uint32_t kFormatVersion = 15;

  // What to serialize into a unified file. `cifp` may be empty (no CIFP section
  // written). `detail` is optional. The graph is always written.
  struct BuildInput {
    const GraphSnapshot* graph = nullptr;
    const std::vector<std::pair<std::string, CifpData>>* cifp = nullptr;  // nullptr => omit
    const NavDetailArchive* detail = nullptr;                             // nullptr => omit
    UnifiedHeader header;
  };

  // Serialize a unified `.bfdb` to `path`. Returns an error if a section fails to
  // encode or the file cannot be written.
  static Result<void> Build(const std::string& path, const BuildInput& input);

  // Read a unified `.bfdb`: parse the header + section table + global pool,
  // decode the graph section, and open the CIFP/detail sections when present.
  // Returns kDataMissing if the file is absent, kCacheCorrupt if malformed, or
  // kFormatMismatch if of an incompatible format version.
  static Result<UnifiedData> Open(const std::string& path);

  // Read only the container header (cycle + provenance) without decoding any
  // section, so a directory of caches can be cataloged cheaply. Same failure
  // modes as Open for the header region.
  static Result<UnifiedHeader> ReadHeader(const std::string& path);
};

}  // namespace bf
