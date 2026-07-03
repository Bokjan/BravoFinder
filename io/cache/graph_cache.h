#pragma once

#include <cstdint>
#include <string>

#include "core/result.h"

namespace bf {

struct GraphSnapshot;  // io/cache/graph_snapshot.h

// The header fields of a `.bfdb`, readable without deserializing the graph.
// Used to catalog a directory of caches by their authoritative cycle/build
// (the filename is only a hint; see bfdb_naming.h).
struct GraphCacheHeader {
  uint32_t cycle = 0;
  uint32_t build = 0;
  std::string program_semver;  // bf version that built this cache
  std::string source_loader;   // loader that produced the data, e.g. "xplane"
};

// Binary serialization of a GraphSnapshot to and from a `.bfdb` file.
//
// The format is explicit, fixed-width, little-endian, and uses IEEE-754 bit
// patterns for floats, so a file produced on one platform (x86-64, ARM) reads
// identically on another. It is deliberately NOT mmap-oriented: values are
// parsed field-by-field back into owning std::vectors. A header carries a magic
// tag, a format version, and the vertex/edge counts so a mismatched or
// truncated file is rejected cleanly via Result rather than crashing.
//
// Named for its payload (the graph), matching the CifpCache / NavDetailCache
// sibling codecs; the "bfdb" name belongs to the on-disk container (extension
// and magic), see bfdb_naming.h / bfdb_inventory.h.
class GraphCache {
 public:
  // The current on-disk format version. Bump whenever the layout changes; older
  // files are then rejected and the user re-runs `bf build`.
  //
  // v2 (M4): added program_semver and source_loader to the header.
  // v3: per-vertex data serialized as one record each (coord + ident + flags +
  //     waypoint kind) and a trailing airport record section (elevation),
  //     replacing the parallel coords/on_network/idents arrays. Extensible: a
  //     new per-vertex field is one more field in the vertex record.
  static constexpr uint32_t kFormatVersion = 3;

  // Serialize `snapshot` to `path`. Returns an error if the file cannot be
  // written or the snapshot exceeds format limits (e.g. > 65535 airway names).
  static Result<void> Build(const std::string& path, const GraphSnapshot& snapshot);

  // Read a `.bfdb` file into a GraphSnapshot. Returns kDataMissing if the file is
  // absent, kCacheCorrupt if malformed/truncated, or kFormatMismatch if of an
  // incompatible format version.
  static Result<GraphSnapshot> Open(const std::string& path);

  // Read only the header (magic, version, cycle/build, provenance strings)
  // without deserializing the graph body, so a directory of caches can be
  // cataloged cheaply. Same failure modes as Open for the header region.
  static Result<GraphCacheHeader> ReadHeader(const std::string& path);
};

}  // namespace bf
