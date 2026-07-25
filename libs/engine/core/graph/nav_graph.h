// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <vector>

#include "core/domain/airway.h"
#include "core/domain/coordinate.h"

namespace bf {

// A directed edge in the navigation graph, stored in compressed-sparse-row
// (CSR) form. `to` is the destination vertex index; `distance_nm` is the
// great-circle length of the segment. `airway_id` indexes into the graph's
// airway-name table (0 is the reserved "DCT" entry for synthetic edges).
// Altitude band and level are kept for constraint filtering.
//
// Fields are sized to keep the struct compact (16 bytes): the edge array is the
// largest structure in the graph and A* traverses it on the hot path, so a
// smaller edge doubles the number that fit in a cache line. `distance_nm` is
// stored as float (single-segment precision to ~2 m, ample); path costs are
// accumulated in double by the search, so no precision is lost across a route.
//
// `level` is a single AirwayLevel value (low/high/both) rather than bit flags:
// the three states are mutually exclusive, so a bitfield would admit a
// nonsensical "high and both" combination. It is serialized as its raw uint8_t
// value (see the static_assert in airway.h pinning those values to the cache
// format).
struct GraphEdge {
  int32_t to = -1;                        // destination vertex index
  float distance_nm = 0.0f;               // great-circle length; searches accumulate in double
  uint16_t airway_id = 0;                 // index into airway-name table (0 = "DCT")
  int16_t base_fl = 0;                    // lowest usable flight level (0 = no limit, e.g. DCT)
  int16_t top_fl = 0;                     // highest usable flight level (0 = no limit)
  AirwayLevel level = AirwayLevel::kLow;  // low/high/both airway structure
};
static_assert(sizeof(GraphEdge) == 16, "GraphEdge is expected to be 16 bytes");

// An immutable directed graph over navigation waypoints, stored as CSR for
// cache-friendly traversal. Vertices are integer indices; each vertex carries
// its coordinate so A* can compute its great-circle heuristic. Build instances
// via GraphBuilder.
class NavGraph {
 public:
  NavGraph() = default;

  int VertexCount() const { return static_cast<int>(coords_.size()); }

  const Coordinate& CoordOf(int vertex) const { return coords_[vertex]; }

  // Edges leaving `vertex`, as a contiguous range [begin, end).
  const GraphEdge* EdgesBegin(int vertex) const { return edges_.data() + offsets_[vertex]; }
  const GraphEdge* EdgesEnd(int vertex) const { return edges_.data() + offsets_[vertex + 1]; }

 private:
  friend class GraphBuilder;

  std::vector<Coordinate> coords_;  // per-vertex position, size = V
  std::vector<int> offsets_;        // CSR row offsets, size = V + 1
  std::vector<GraphEdge> edges_;    // CSR edge array, size = E
};

}  // namespace bf
