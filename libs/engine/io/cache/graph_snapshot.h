// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/domain/fixed_ident.h"
#include "core/domain/ident.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/domain/waypoint.h"
#include "core/graph/nav_graph.h"

namespace bf {

// A flat, self-contained snapshot of the route-graph data a NavDatabase needs
// to answer queries, decoupled from GraphBuilder's internal indexing. GraphCodec
// encodes and decodes this snapshot as one section of a unified `.bfdb`;
// GraphBuilder converts to/from it. Unlike CifpArchive / NavDetailArchive (which
// are live, queryable views), this is a passive data carrier with no lookup
// methods of its own.
//
// This holds ONLY graph data -- no AIRAC/provenance metadata. Cycle, source
// loader, program version and data dir live once in the unified container header
// (see unified_cache.h), not per section.
//
// The three lookup maps are NOT part of the snapshot: they are rebuilt from
// `idents` on load (an unordered_map is not portably serializable and costs more
// in RAM than the arrays it indexes).
struct GraphSnapshot {
  int first_airport_vertex = 0;

  std::vector<Coordinate> coords;     // per-vertex position, size V
  std::vector<int> offsets;           // CSR row offsets, size V + 1
  std::vector<GraphEdge> edges;       // CSR edge array, size E
  std::vector<uint8_t> has_outbound;  // per-vertex: >=1 outbound airway edge (0/1), size V
  std::vector<uint8_t> has_inbound;   // per-vertex: >=1 inbound airway edge (0/1), size V
  std::vector<FixedIdent> idents;     // per-vertex (ident, region), size V
  std::vector<WaypointKind> kinds;    // per-vertex kind (fix/VOR/NDB/DME), size V
  std::vector<std::string> airway_names;
  MoraGrid mora;
  std::vector<MsaSector> msa;

  // Airport-only attributes, indexed by airport ordinal (vertex index minus
  // first_airport_vertex). Size = V - first_airport_vertex.
  std::vector<int> airport_elevations_ft;
};

}  // namespace bf
