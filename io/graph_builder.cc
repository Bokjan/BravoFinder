#include "io/graph_builder.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
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
  auto airway_id_for = [&](const std::string& name) {
    auto it = name_to_id.find(name);
    if (it != name_to_id.end()) {
      return it->second;
    }
    const int id = static_cast<int>(airway_names_.size());
    airway_names_.push_back(name);
    name_to_id.emplace(name, id);
    return id;
  };

  // --- Adjacency list (built first, then flattened to CSR). ---
  std::vector<std::vector<GraphEdge>> adj(total);
  auto add_edge = [&](int from, int to, int airway_id, const AirwaySegment& s) {
    const double dist = graph_.coords_[from].DistanceTo(graph_.coords_[to]);
    adj[from].push_back(GraphEdge{to, dist, airway_id, static_cast<int16_t>(s.base_fl),
                                  static_cast<int16_t>(s.top_fl)});
  };

  for (const AirwayConnection& conn : data.airways) {
    auto from_it = ident_index_.find(conn.from);
    auto to_it = ident_index_.find(conn.to);
    if (from_it == ident_index_.end() || to_it == ident_index_.end()) {
      continue;  // endpoint not in dataset; skip the segment
    }
    const int from = from_it->second;
    const int to = to_it->second;
    const int id = airway_id_for(conn.segment.name);
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
}

int GraphBuilder::VertexByIdent(const std::string& ident) const {
  auto it = ident_first_.find(ident);
  return it == ident_first_.end() ? -1 : it->second;
}

int GraphBuilder::VertexByAirport(const std::string& icao) const {
  auto it = airport_index_.find(icao);
  return it == airport_index_.end() ? -1 : it->second;
}

const std::string& GraphBuilder::AirwayName(int airway_id) const {
  return airway_names_[airway_id];
}

}  // namespace bf
