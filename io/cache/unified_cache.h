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
  uint32_t cycle = 0;          // AIRAC cycle, e.g. 2601; 0 means no provenance
  std::string program_semver;  // bf version that built this cache
  std::string source_loader;   // loader that produced the data, e.g. "xplane12"
  std::string data_dir;        // the data dir used at build time (route default)
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
//                  program_semver, source_loader, data_dir, pool_len
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
  static constexpr uint32_t kFormatVersion = 4;

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
