#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/domain/ident.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/domain/waypoint.h"
#include "core/graph/nav_graph.h"
#include "core/result.h"

namespace bf {

// A flat, self-contained snapshot of everything a NavDatabase needs to answer
// queries, decoupled from GraphBuilder's internal indexing. BfdbCache reads and
// writes this image; GraphBuilder converts to/from it. The three lookup maps
// are NOT part of the image: they are rebuilt from `idents` on load (an
// unordered_map is not portably serializable and costs more in RAM than the
// arrays it indexes).
struct BfdbImage {
  uint32_t cycle = 0;
  uint32_t build = 0;
  std::string program_semver;  // bf version that built this cache
  std::string source_loader;   // loader that produced the data, e.g. "xplane"
  int first_airport_vertex = 0;
  std::string data_dir;  // the data dir used at build time (route default)

  std::vector<Coordinate> coords;   // per-vertex position, size V
  std::vector<int> offsets;         // CSR row offsets, size V + 1
  std::vector<GraphEdge> edges;     // CSR edge array, size E
  std::vector<bool> on_network;     // per-vertex airway membership, size V
  std::vector<Ident> idents;        // per-vertex (ident, region), size V
  std::vector<WaypointKind> kinds;  // per-vertex kind (fix/VOR/NDB/DME), size V
  std::vector<std::string> airway_names;
  MoraGrid mora;
  std::vector<MsaSector> msa;

  // Airport-only attributes, indexed by airport ordinal (vertex index minus
  // first_airport_vertex). Size = V - first_airport_vertex.
  std::vector<int> airport_elevations_ft;
};

// Binary serialization of a BfdbImage to and from a `.bfdb` file.
//
// The format is explicit, fixed-width, little-endian, and uses IEEE-754 bit
// patterns for floats, so a file produced on one platform (x86-64, ARM) reads
// identically on another. It is deliberately NOT mmap-oriented: values are
// parsed field-by-field back into owning std::vectors. A header carries a magic
// tag, a format version, and the vertex/edge counts so a mismatched or
// truncated file is rejected cleanly via Result rather than crashing.
class BfdbCache {
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

  // Serialize `image` to `path`. Returns an error if the file cannot be written
  // or the image exceeds format limits (e.g. > 65535 airway names).
  static Result<void> Write(const std::string& path, const BfdbImage& image);

  // Read a `.bfdb` file into a BfdbImage. Returns kDataMissing if the file is
  // absent, malformed, truncated, or of an incompatible format version.
  static Result<BfdbImage> Read(const std::string& path);
};

}  // namespace bf
