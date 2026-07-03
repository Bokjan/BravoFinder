#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/domain/ident.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/domain/waypoint.h"
#include "core/graph/nav_graph.h"

namespace bf {

// A flat, self-contained snapshot of everything a NavDatabase needs to answer
// queries, decoupled from GraphBuilder's internal indexing. GraphCache reads and
// writes this snapshot; GraphBuilder converts to/from it. Unlike CifpArchive /
// NavDetailArchive (which are live, queryable views), this is a passive data
// carrier with no lookup methods of its own.
//
// The three lookup maps are NOT part of the snapshot: they are rebuilt from
// `idents` on load (an unordered_map is not portably serializable and costs more
// in RAM than the arrays it indexes).
struct GraphSnapshot {
  uint32_t cycle = 0;
  uint32_t build = 0;
  std::string program_semver;  // bf version that built this cache
  std::string source_loader;   // loader that produced the data, e.g. "xplane"
  int first_airport_vertex = 0;
  std::string data_dir;  // the data dir used at build time (route default)

  std::vector<Coordinate> coords;   // per-vertex position, size V
  std::vector<int> offsets;         // CSR row offsets, size V + 1
  std::vector<GraphEdge> edges;     // CSR edge array, size E
  std::vector<uint8_t> on_network;  // per-vertex airway membership (0/1), size V
  std::vector<Ident> idents;        // per-vertex (ident, region), size V
  std::vector<WaypointKind> kinds;  // per-vertex kind (fix/VOR/NDB/DME), size V
  std::vector<std::string> airway_names;
  MoraGrid mora;
  std::vector<MsaSector> msa;

  // Airport-only attributes, indexed by airport ordinal (vertex index minus
  // first_airport_vertex). Size = V - first_airport_vertex.
  std::vector<int> airport_elevations_ft;
};

}  // namespace bf
