#include "io/graph_builder.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bf {

namespace {

// A coarse spatial index that buckets vertices by integer (lat, lon) degree.
// Used to find waypoints near an airport without scanning the whole dataset.
class DegreeGrid {
 public:
  void Insert(int vertex, const Coordinate& c) {
    cells_[Key(c.latitude, c.longitude)].push_back(vertex);
  }

  // Collect candidate vertices within `radius_deg` cells of the position.
  std::vector<int> Near(const Coordinate& c, int radius_deg) const {
    std::vector<int> out;
    const int lat0 = static_cast<int>(std::floor(c.latitude));
    const int lon0 = static_cast<int>(std::floor(c.longitude));
    for (int dlat = -radius_deg; dlat <= radius_deg; ++dlat) {
      for (int dlon = -radius_deg; dlon <= radius_deg; ++dlon) {
        auto it = cells_.find(CellKey(lat0 + dlat, lon0 + dlon));
        if (it != cells_.end()) {
          out.insert(out.end(), it->second.begin(), it->second.end());
        }
      }
    }
    return out;
  }

 private:
  static long Key(double lat, double lon) {
    return CellKey(static_cast<int>(std::floor(lat)), static_cast<int>(std::floor(lon)));
  }
  static long CellKey(int lat, int lon) { return static_cast<long>(lat + 90) * 1000 + (lon + 180); }
  std::unordered_map<long, std::vector<int>> cells_;
};

}  // namespace

GraphBuilder::GraphBuilder(const NavData& data, int airport_dct_count) {
  const int waypoint_count = static_cast<int>(data.waypoints.size());
  const int airport_count = static_cast<int>(data.airports.size());
  const int total = waypoint_count + airport_count;

  // --- Vertices: waypoints first, then airports. ---
  graph_.coords_.reserve(total);
  idents_.reserve(total);
  DegreeGrid grid;
  for (int i = 0; i < waypoint_count; ++i) {
    const Waypoint& w = data.waypoints[i];
    graph_.coords_.push_back(w.coord);
    idents_.push_back(w.ident);
    ident_index_.emplace(w.ident, i);
    ident_first_.emplace(w.ident.ident, i);
    grid.Insert(i, w.coord);
  }
  for (int i = 0; i < airport_count; ++i) {
    const Airport& a = data.airports[i];
    const int v = waypoint_count + i;
    graph_.coords_.push_back(a.coord);
    idents_.push_back(Ident(a.icao, a.region));
    airport_index_.emplace(a.icao, v);
  }

  // --- Airway-name table; "DCT" reserved at index 0 for synthetic edges. ---
  airway_names_.push_back("DCT");
  std::unordered_map<std::string, int> name_to_id;
  auto airway_id_for = [&](const std::string& name) -> uint16_t {
    auto it = name_to_id.find(name);
    if (it != name_to_id.end()) {
      return static_cast<uint16_t>(it->second);
    }
    const int id = static_cast<int>(airway_names_.size());
    // airway_id is a uint16 in GraphEdge; guard against overflow. Real AIRAC
    // data has ~12k distinct airway names, far under the limit, so this is a
    // fuse rather than an expected condition.
    if (id > 0xFFFF) {
      return 0;  // fall back to "DCT"; should never happen with real data
    }
    airway_names_.push_back(name);
    name_to_id.emplace(name, id);
    return static_cast<uint16_t>(id);
  };

  // --- Adjacency list (built first, then flattened to CSR). ---
  std::vector<std::vector<GraphEdge>> adj(total);
  auto add_edge = [&](int from, int to, uint16_t airway_id, const AirwaySegment& s) {
    const double dist = graph_.coords_[from].DistanceTo(graph_.coords_[to]);
    const uint8_t flags = s.level == AirwayLevel::kHigh ? kEdgeHigh : 0;
    adj[from].push_back(GraphEdge{to, static_cast<float>(dist), airway_id,
                                  static_cast<int16_t>(s.base_fl), static_cast<int16_t>(s.top_fl),
                                  flags});
  };

  for (const AirwayConnection& conn : data.airways) {
    auto from_it = ident_index_.find(conn.from);
    auto to_it = ident_index_.find(conn.to);
    if (from_it == ident_index_.end() || to_it == ident_index_.end()) {
      continue;  // endpoint not in dataset; skip the segment
    }
    const int from = from_it->second;
    const int to = to_it->second;
    const uint16_t id = airway_id_for(conn.segment.name);
    // Honor directionality: kForward = from->to only, kBackward = to->from only,
    // kBoth = both directions.
    if (conn.segment.direction != AirwayDirection::kBackward) {
      add_edge(from, to, id, conn.segment);
    }
    if (conn.segment.direction != AirwayDirection::kForward) {
      add_edge(to, from, id, conn.segment);
    }
  }

  // --- Connect airports to nearest waypoints with bidirectional DCT edges. ---
  // Only connect to waypoints that actually participate in the enroute airway
  // network. Terminal-area fixes (approach/SID/STAR points) are geographically
  // closest to an airport but are dead ends here until procedures are modeled,
  // so connecting to them would strand the airport off the network.
  std::vector<bool> on_network(total, false);
  for (int v = 0; v < total; ++v) {
    if (!adj[v].empty()) {
      on_network[v] = true;
    }
  }
  AirwaySegment dct;  // default-constructed: name empty, FL 0..0
  for (int i = 0; i < airport_count; ++i) {
    const Airport& a = data.airports[i];
    const int v = waypoint_count + i;
    // Expand the search radius until enough on-network candidates are found.
    std::vector<int> candidates;
    for (int radius = 2; radius <= 16 && candidates.empty(); radius += 2) {
      for (int cand : grid.Near(a.coord, radius)) {
        if (on_network[cand]) {
          candidates.push_back(cand);
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(), [&](int x, int y) {
      return a.coord.DistanceTo(graph_.coords_[x]) < a.coord.DistanceTo(graph_.coords_[y]);
    });
    const int k = std::min<int>(airport_dct_count, static_cast<int>(candidates.size()));
    for (int j = 0; j < k; ++j) {
      add_edge(v, candidates[j], 0, dct);
      add_edge(candidates[j], v, 0, dct);
    }
  }

  // --- Flatten adjacency to CSR. ---
  graph_.offsets_.resize(total + 1, 0);
  for (int v = 0; v < total; ++v) {
    graph_.offsets_[v + 1] = graph_.offsets_[v] + static_cast<int>(adj[v].size());
  }
  graph_.edges_.reserve(graph_.offsets_[total]);
  for (int v = 0; v < total; ++v) {
    graph_.edges_.insert(graph_.edges_.end(), adj[v].begin(), adj[v].end());
  }

  // Persist the on-network flags (computed before DCT edges were added) so
  // procedure wiring can tell true enroute vertices from terminal-only fixes.
  on_network_ = std::move(on_network);
  first_airport_vertex_ = waypoint_count;
}

int GraphBuilder::VertexByIdent(const Ident& ident) const {
  auto it = ident_index_.find(ident);
  return it == ident_index_.end() ? -1 : it->second;
}

bool GraphBuilder::OnNetwork(int vertex) const {
  return vertex >= 0 && vertex < static_cast<int>(on_network_.size()) && on_network_[vertex];
}

std::vector<int> GraphBuilder::NearestOnNetwork(const Coordinate& coord, int count) const {
  std::vector<int> candidates;
  const int v_count = graph_.VertexCount();
  candidates.reserve(256);
  for (int v = 0; v < v_count; ++v) {
    if (on_network_[v]) {
      candidates.push_back(v);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [&](int x, int y) {
    return coord.DistanceTo(graph_.coords_[x]) < coord.DistanceTo(graph_.coords_[y]);
  });
  if (static_cast<int>(candidates.size()) > count) {
    candidates.resize(count);
  }
  return candidates;
}

int GraphBuilder::VertexByIdent(const std::string& ident) const {
  auto it = ident_first_.find(ident);
  return it == ident_first_.end() ? -1 : it->second;
}

int GraphBuilder::VertexByAirport(const std::string& icao) const {
  auto it = airport_index_.find(icao);
  return it == airport_index_.end() ? -1 : it->second;
}

bool GraphBuilder::IsAirport(int vertex) const {
  return vertex >= first_airport_vertex_ && vertex < graph_.VertexCount();
}

const std::string& GraphBuilder::AirwayName(int airway_id) const {
  return airway_names_[airway_id];
}

void GraphBuilder::RebuildIndices() {
  const int v_count = static_cast<int>(idents_.size());
  ident_index_.clear();
  ident_first_.clear();
  airport_index_.clear();
  ident_index_.reserve(v_count);
  ident_first_.reserve(v_count);
  // Waypoints occupy [0, first_airport_vertex_); airports the tail. Both are
  // reachable by (ident, region); only waypoints seed the ident-only and
  // airports the ICAO lookup, matching the constructor's original wiring.
  for (int i = 0; i < first_airport_vertex_; ++i) {
    ident_index_.emplace(idents_[i], i);
    ident_first_.emplace(idents_[i].ident, i);
  }
  for (int v = first_airport_vertex_; v < v_count; ++v) {
    airport_index_.emplace(idents_[v].ident, v);
  }
}

GraphBuilder GraphBuilder::FromImage(BfdbImage&& image) {
  GraphBuilder b;
  b.graph_.coords_ = std::move(image.coords);
  b.graph_.offsets_ = std::move(image.offsets);
  b.graph_.edges_ = std::move(image.edges);
  b.idents_ = std::move(image.idents);
  b.on_network_ = std::move(image.on_network);
  b.airway_names_ = std::move(image.airway_names);
  b.first_airport_vertex_ = image.first_airport_vertex;
  b.RebuildIndices();
  return b;
}

BfdbImage GraphBuilder::ToImage(const std::string& data_dir, uint32_t cycle, uint32_t build) const {
  BfdbImage image;
  image.cycle = cycle;
  image.build = build;
  image.first_airport_vertex = first_airport_vertex_;
  image.data_dir = data_dir;
  image.coords = graph_.coords_;
  image.offsets = graph_.offsets_;
  image.edges = graph_.edges_;
  image.on_network = on_network_;
  image.idents = idents_;
  image.airway_names = airway_names_;
  // mora/msa are owned by NavDatabase, not the builder; the caller fills them.
  return image;
}

}  // namespace bf
