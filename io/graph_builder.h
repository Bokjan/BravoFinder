#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "core/graph/nav_graph.h"
#include "io/nav_data.h"

namespace bf {

// Builds an immutable NavGraph (CSR) from a loaded NavData. Resolves airway
// endpoints by (ident, region) to vertex indices, emits directed edges honoring
// each segment's direction, and (for M1) connects each airport to its nearest
// waypoints with synthetic direct (DCT) edges.
//
// The builder also records the mapping needed to translate user-facing names
// (airport ICAO, waypoint ident) into vertex indices for queries.
class GraphBuilder {
 public:
  // Build the graph from `data`. Airports are connected with up to
  // `airport_dct_count` direct edges to their nearest waypoints.
  explicit GraphBuilder(const NavData& data, int airport_dct_count = 5);

  const NavGraph& graph() const { return graph_; }

  // Resolve a waypoint by ident alone (first match across regions), or -1.
  int VertexByIdent(const std::string& ident) const;

  // Resolve a waypoint by its full (ident, region) key, or -1. Preferred over
  // the ident-only lookup when the region is known (procedure fixes carry it),
  // since idents are not globally unique.
  int VertexByIdent(const Ident& ident) const;

  // Resolve an airport by ICAO code to its vertex index, or -1.
  int VertexByAirport(const std::string& icao) const;

  // Whether `vertex` participates in the enroute airway network (has at least
  // one airway edge, as opposed to only synthetic DCT edges or none). Terminal
  // fixes that only appear in procedures are not on-network until procedures
  // wire them in, so this distinguishes usable connection fixes.
  bool OnNetwork(int vertex) const;

  // Find up to `count` on-network vertices nearest to `coord`, ordered nearest
  // first. Used as the DCT fallback when an airport has no procedure data.
  std::vector<int> NearestOnNetwork(const Coordinate& coord, int count) const;

  // The airway name for an edge's airway_id, or "DCT" for synthetic edges.
  const std::string& AirwayName(int airway_id) const;

  // Vertex metadata for result construction.
  const Ident& IdentOf(int vertex) const { return idents_[vertex]; }

 private:
  NavGraph graph_;
  std::vector<Ident> idents_;     // per-vertex ident, size = V
  std::vector<bool> on_network_;  // per-vertex: participates in an airway, size = V
  std::vector<std::string> airway_names_;
  std::unordered_map<Ident, int> ident_index_;          // (ident,region) -> vertex
  std::unordered_map<std::string, int> ident_first_;    // ident -> first vertex
  std::unordered_map<std::string, int> airport_index_;  // ICAO -> vertex
};

}  // namespace bf
