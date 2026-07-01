#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/graph/nav_graph.h"
#include "io/cache/bfdb_cache.h"
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

  // Assemble a builder directly from a deserialized cache image (the `bf route
  // --db` path): the graph arrays and vertex metadata are moved in, and the
  // three lookup maps are rebuilt from the idents. Skips all parsing and graph
  // construction.
  static GraphBuilder FromImage(BfdbImage&& image);

  // Export the built graph and metadata as a cache image for BfdbCache::Write
  // (the `bf build` path). `data_dir`/`cycle`/`build` become the image header.
  BfdbImage ToImage(const std::string& data_dir, uint32_t cycle, uint32_t build) const;

  const NavGraph& graph() const { return graph_; }

  // Resolve a waypoint by ident alone (first match across regions), or -1.
  int VertexByIdent(const std::string& ident) const;

  // Resolve a waypoint by its full (ident, region) key, or -1. Preferred over
  // the ident-only lookup when the region is known (procedure fixes carry it),
  // since idents are not globally unique.
  int VertexByIdent(const Ident& ident) const;

  // Resolve an airport by ICAO code to its vertex index, or -1.
  int VertexByAirport(const std::string& icao) const;

  // Whether `vertex` is an airport node (as opposed to a waypoint/navaid).
  // Airports occupy the contiguous tail of the vertex range. Airports are valid
  // route endpoints but must never be used as intermediate transit nodes, since
  // their synthetic DCT links would otherwise let a search cut through an
  // unrelated airport.
  bool IsAirport(int vertex) const;

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
  // For FromImage: constructs an empty builder to be populated from an image.
  GraphBuilder() = default;

  // Rebuild the three lookup maps from idents_ / first_airport_vertex_. Used
  // after the vertex metadata is in place (both build paths converge here).
  void RebuildIndices();

  NavGraph graph_;
  std::vector<Ident> idents_;     // per-vertex ident, size = V
  std::vector<bool> on_network_;  // per-vertex: participates in an airway, size = V
  int first_airport_vertex_ = 0;  // vertices [this, V) are airports
  std::vector<std::string> airway_names_;
  std::unordered_map<Ident, int> ident_index_;          // (ident,region) -> vertex
  std::unordered_map<std::string, int> ident_first_;    // ident -> first vertex
  std::unordered_map<std::string, int> airport_index_;  // ICAO -> vertex
};

}  // namespace bf
