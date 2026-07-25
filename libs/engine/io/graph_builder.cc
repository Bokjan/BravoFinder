// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/graph_builder.h"

#include <algorithm>
#include <cassert>
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
  kinds_.reserve(total);
  airport_elevations_ft_.reserve(airport_count);
  DegreeGrid grid;
  for (int i = 0; i < waypoint_count; ++i) {
    const Waypoint& w = data.waypoints[i];
    graph_.coords_.push_back(w.coord);
    idents_.push_back(FixedIdent::FromIdent(w.ident));
    kinds_.push_back(w.kind);
    grid.Insert(i, w.coord);
  }
  for (int i = 0; i < airport_count; ++i) {
    const Airport& a = data.airports[i];
    graph_.coords_.push_back(a.coord);
    idents_.push_back(FixedIdent::FromParts(a.icao, a.region));
    kinds_.push_back(WaypointKind::kFix);  // airports have no navaid kind
    airport_elevations_ft_.push_back(a.elevation_ft);
  }
  first_airport_vertex_ = waypoint_count;
  // Build the sorted lookup indices now: airway resolution below queries them by
  // (ident, region). Both build paths (here and FromSnapshot) go through this.
  RebuildIndices();

  // --- Airway-name table; "DCT" reserved at index 0 for synthetic edges. ---
  airway_names_.push_back("DCT");
  std::unordered_map<std::string, int> name_to_id;
  auto airway_id_for = [&](const std::string& name) -> int {
    auto it = name_to_id.find(name);
    if (it != name_to_id.end()) {
      return it->second;
    }
    const int id = static_cast<int>(airway_names_.size());
    // Overflow guard for the uint16 airway_id space (see airway_overflow()); real
    // data (~12k names) never hits this. Return a -1 sentinel so the caller drops
    // the edge rather than silently remapping it to a synthetic DCT.
    if (id > 0xFFFF) {
      airway_overflow_ = true;
      return -1;  // sentinel: caller skips the edge rather than emitting a wrong DCT
    }
    airway_names_.push_back(name);
    name_to_id.emplace(name, id);
    return id;
  };

  // --- Adjacency list (built first, then flattened to CSR). ---
  std::vector<std::vector<GraphEdge>> adj(total);
  auto add_edge = [&](int from, int to, int airway_id, const AirwaySegment& s) {
    if (airway_id < 0) {
      return;  // airway overflow: drop this edge rather than emit a wrong DCT
    }
    const double dist = graph_.coords_[from].DistanceTo(graph_.coords_[to]);
    adj[from].push_back(GraphEdge{to, static_cast<float>(dist), static_cast<uint16_t>(airway_id),
                                  static_cast<int16_t>(s.base_fl), static_cast<int16_t>(s.top_fl),
                                  s.level});
  };

  for (const AirwayConnection& conn : data.airways) {
    const int from = VertexByIdent(conn.from);
    const int to = VertexByIdent(conn.to);
    if (from < 0 || to < 0) {
      continue;  // endpoint not in dataset; skip the segment
    }
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
  //
  // Two per-vertex flags are derived here, from airway edges only (before the
  // DCT edges below are added): has_outbound (>=1 outgoing airway edge) and
  // has_inbound (>=1 incoming airway edge). A SID must hand off to an outbound
  // fix; a STAR must be picked up at an inbound fix. A forward-only airway that
  // dead-ends at a fix (e.g. a STAR entry gate) leaves that fix inbound-only.
  std::vector<uint8_t> has_outbound(total, 0);
  std::vector<uint8_t> has_inbound(total, 0);
  for (int v = 0; v < total; ++v) {
    if (!adj[v].empty()) {
      has_outbound[v] = 1;
      for (const GraphEdge& e : adj[v]) {
        has_inbound[e.to] = 1;
      }
    }
  }
  // A synthetic DCT leg is a direct segment with no airway structure, usable at
  // any altitude: model it as kBoth so LevelPreferenceConstraint never penalizes
  // the airport-to-network connectors regardless of the requested level. (The
  // altitude-band constraint already exempts it via base_fl==0 && top_fl==0.)
  AirwaySegment dct;  // default-constructed: name empty, FL 0..0
  dct.level = AirwayLevel::kBoth;
  for (int i = 0; i < airport_count; ++i) {
    const Airport& a = data.airports[i];
    const int v = waypoint_count + i;
    // Expand the search radius until enough on-network candidates are found.
    // NOTE: these airport DCT edges are effectively dead in routing -- airport
    // vertices are node_blocked for every search role, and search endpoints are
    // always seeded connection fixes, so the edges added just below are never
    // traversed. They are kept for graph completeness/inspection. Do NOT
    // "widen" the candidate filter to has_outbound || has_inbound to fix
    // asymmetry: that only adds more unreachable dead edges. The real fix is to
    // rethink the airport connectivity model (or drop these edges); the DCT
    // fallback path (NearestOnNetwork) already selects candidates directionally.
    std::vector<int> candidates;
    for (int radius = 2; radius <= 16 && candidates.empty(); radius += 2) {
      for (int cand : grid.Near(a.coord, radius)) {
        if (has_outbound[cand]) {
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
  // procedure wiring can tell true enroute vertices from terminal-only fixes,
  // and can pick the right direction: outbound for SID, inbound for STAR.
  has_outbound_ = std::move(has_outbound);
  has_inbound_ = std::move(has_inbound);
}

int GraphBuilder::VertexByIdent(const FixedIdent& key) const {
  auto it = std::lower_bound(
      ident_index_.begin(), ident_index_.end(), key,
      [](const std::pair<FixedIdent, int>& e, const FixedIdent& k) { return e.first < k; });
  if (it != ident_index_.end() && it->first == key) {
    return it->second;
  }
  return -1;
}

int GraphBuilder::VertexByIdent(const Ident& ident) const {
  // A query string longer than the fixed caps cannot match any stored key
  // (real idents are <= 5 / regions <= 2). Short-circuit so FromIdent's
  // build-side overflow assert never fires on a legitimate over-long query.
  if (ident.ident.size() > FixedIdent::kIdentCap || ident.region.size() > FixedIdent::kRegionCap) {
    return -1;
  }
  return VertexByIdent(FixedIdent::FromIdent(ident));
}

bool GraphBuilder::OnNetwork(int vertex) const { return HasOutbound(vertex) || HasInbound(vertex); }

bool GraphBuilder::HasOutbound(int vertex) const {
  return vertex >= 0 && vertex < static_cast<int>(has_outbound_.size()) && has_outbound_[vertex];
}

bool GraphBuilder::HasInbound(int vertex) const {
  return vertex >= 0 && vertex < static_cast<int>(has_inbound_.size()) && has_inbound_[vertex];
}

std::vector<int> GraphBuilder::NearestOnNetwork(const Coordinate& coord, int count,
                                                bool inbound) const {
  if (count <= 0) {
    return {};
  }
  const int v_count = graph_.VertexCount();
  const std::vector<uint8_t>& mask = inbound ? has_inbound_ : has_outbound_;
  // Score each on-network candidate with its distance computed exactly once; a
  // sort comparator would recompute DistanceTo O(log V) times per element. Pairs
  // sort by (distance, vertex), so ties break deterministically on vertex id.
  std::vector<std::pair<double, int>> scored;
  scored.reserve(256);
  for (int v = 0; v < v_count; ++v) {
    if (mask[v]) {
      scored.emplace_back(coord.DistanceTo(graph_.coords_[v]), v);
    }
  }
  const size_t k = std::min(static_cast<size_t>(count), scored.size());
  std::partial_sort(scored.begin(), scored.begin() + k, scored.end());
  std::vector<int> out;
  out.reserve(k);
  for (size_t i = 0; i < k; ++i) {
    out.push_back(scored[i].second);
  }
  return out;
}

std::vector<int> GraphBuilder::VerticesByIdent(const std::string& ident) const {
  if (ident.size() > FixedIdentNoRegion::kCap) {
    return {};  // longer than any stored ident -> no match (see VertexByIdent)
  }
  const FixedIdentNoRegion key = FixedIdentNoRegion::From(ident);
  auto lo = std::lower_bound(ident_all_.begin(), ident_all_.end(), key,
                             [](const std::pair<FixedIdentNoRegion, int>& e,
                                const FixedIdentNoRegion& k) { return e.first < k; });
  std::vector<int> out;
  for (auto it = lo; it != ident_all_.end() && it->first == key; ++it) {
    out.push_back(it->second);
  }
  return out;
}

int GraphBuilder::VertexByAirport(const std::string& icao) const {
  if (icao.size() > FixedIdentNoRegion::kCap) {
    return -1;  // longer than any stored ICAO -> no match (see VertexByIdent)
  }
  const FixedIdentNoRegion key = FixedIdentNoRegion::From(icao);
  auto it = std::lower_bound(airport_index_.begin(), airport_index_.end(), key,
                             [](const std::pair<FixedIdentNoRegion, int>& e,
                                const FixedIdentNoRegion& k) { return e.first < k; });
  if (it != airport_index_.end() && it->first == key) {
    return it->second;
  }
  return -1;
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
  ident_all_.clear();
  airport_index_.clear();
  ident_index_.reserve(first_airport_vertex_);
  ident_all_.reserve(first_airport_vertex_);
  airport_index_.reserve(v_count - first_airport_vertex_);
  // Waypoints occupy [0, first_airport_vertex_); airports the tail. Only
  // waypoints seed the (ident,region) and ident-all lookups; only airports seed
  // the ICAO lookup, matching the original wiring. Each index is filled in
  // vertex order, then sorted once for binary-search lookup.
  for (int i = 0; i < first_airport_vertex_; ++i) {
    ident_index_.emplace_back(idents_[i], i);
    ident_all_.emplace_back(FixedIdentNoRegion::From(idents_[i].IdentView()), i);
  }
  for (int v = first_airport_vertex_; v < v_count; ++v) {
    airport_index_.emplace_back(FixedIdentNoRegion::From(idents_[v].IdentView()), v);
  }
  auto by_key = [](const auto& a, const auto& b) {
    if (a.first < b.first) {
      return true;
    }
    if (b.first < a.first) {
      return false;
    }
    return a.second < b.second;  // stable secondary order: ascending vertex
  };
  std::sort(ident_index_.begin(), ident_index_.end(), by_key);
  std::sort(ident_all_.begin(), ident_all_.end(), by_key);
  std::sort(airport_index_.begin(), airport_index_.end(), by_key);

  // The lookups (VertexByIdent / VerticesByIdent / VertexByAirport) rely on
  // binary search, which is silently wrong on an unsorted range. Assert the
  // key-only ordering the lookups actually use (the vertex tie-break in by_key
  // is irrelevant to binary search). This is the single write site both build
  // paths converge on, so one assert here guards every future change; it is a
  // debug-only check (compiled out in release) over an already-sorted range.
  auto key_sorted = [](const auto& vec) {
    return std::is_sorted(vec.begin(), vec.end(),
                          [](const auto& a, const auto& b) { return a.first < b.first; });
  };
  assert(key_sorted(ident_index_) && "ident_index_ not sorted by key");
  assert(key_sorted(ident_all_) && "ident_all_ not sorted by key");
  assert(key_sorted(airport_index_) && "airport_index_ not sorted by key");
  (void)key_sorted;  // silence unused-variable warning in release (NDEBUG)
}

GraphBuilder GraphBuilder::FromSnapshot(GraphSnapshot&& snapshot) {
  GraphBuilder b;
  b.graph_.coords_ = std::move(snapshot.coords);
  b.graph_.offsets_ = std::move(snapshot.offsets);
  b.graph_.edges_ = std::move(snapshot.edges);
  b.idents_ = std::move(snapshot.idents);
  b.has_outbound_ = std::move(snapshot.has_outbound);
  b.has_inbound_ = std::move(snapshot.has_inbound);
  b.kinds_ = std::move(snapshot.kinds);
  b.airport_elevations_ft_ = std::move(snapshot.airport_elevations_ft);
  b.airway_names_ = std::move(snapshot.airway_names);
  b.first_airport_vertex_ = snapshot.first_airport_vertex;
  b.RebuildIndices();
  return b;
}

GraphSnapshot GraphBuilder::ToSnapshot() const {
  GraphSnapshot snapshot;
  snapshot.first_airport_vertex = first_airport_vertex_;
  snapshot.coords = graph_.coords_;
  snapshot.offsets = graph_.offsets_;
  snapshot.edges = graph_.edges_;
  snapshot.has_outbound = has_outbound_;
  snapshot.has_inbound = has_inbound_;
  snapshot.idents = idents_;
  snapshot.kinds = kinds_;
  snapshot.airport_elevations_ft = airport_elevations_ft_;
  snapshot.airway_names = airway_names_;
  // mora/msa are owned by NavDatabase, not the builder; the caller fills them.
  return snapshot;
}

int GraphBuilder::ElevationOf(int vertex) const {
  if (vertex < first_airport_vertex_ || vertex >= static_cast<int>(idents_.size())) {
    return 0;
  }
  return airport_elevations_ft_[vertex - first_airport_vertex_];
}

}  // namespace bf
