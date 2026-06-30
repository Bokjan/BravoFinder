#pragma once

#include <cstdint>
#include <vector>

#include "core/domain/coordinate.h"

namespace bf {

// A directed edge in the navigation graph, stored in compressed-sparse-row
// (CSR) form. `to` is the destination vertex index; `distance_nm` is the
// great-circle length of the segment. `airway_id` indexes into the graph's
// airway-name table (-1 for a synthetic direct/DCT edge). Altitude band is kept
// for constraint filtering in a later milestone.
struct GraphEdge {
  int to = -1;
  double distance_nm = 0.0;
  int airway_id = -1;
  int16_t base_fl = 0;
  int16_t top_fl = 0;
};

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
