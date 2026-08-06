// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/build/graph_builder.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/domain/nav_tokens.h"

namespace bf {

// A coarse spatial index that buckets waypoints by integer (lat, lon) degree.
// Kept resident on the GraphBuilder so NearestOnNetwork avoids an O(V) scan per
// route. Defined here (out-of-line) so the index internals stay out of
// graph_builder.h.
//
// Storage is three sorted/contiguous arrays (not a hash map): the cell keys are
// sorted unique, a parallel offset array marks each cell's id range, and all
// vertex ids live in one flattened array. This mirrors the builder's other
// lookup indices (ident_index_ etc.) and is both smaller (~1.5 MB vs ~3.9 MB
// for unordered_map -- no per-cell vector control block, no hash nodes, no
// bucket array) and cache-friendly: Near() binary-searches a cell and scans a
// contiguous id range, with zero per-cell mallocs.
class GraphBuilder::DegreeGrid {
 public:
  // Construct from (cell_key, vertex) pairs. Sorts by key and flattens vertex
  // ids so Near() can binary-search a cell and scan a contiguous id range.
  explicit DegreeGrid(std::vector<std::pair<int64_t, int>> entries) {
    std::sort(entries.begin(), entries.end());  // by key, then by vertex id
    keys_.reserve(entries.size());              // worst case: one cell per entry; usually far fewer
    ids_.reserve(entries.size());
    for (size_t i = 0; i < entries.size();) {
      const int64_t key = entries[i].first;
      keys_.push_back(key);
      starts_.push_back(static_cast<int>(ids_.size()));
      do {
        ids_.push_back(entries[i].second);
        ++i;
      } while (i < entries.size() && entries[i].first == key);
    }
    starts_.push_back(static_cast<int>(ids_.size()));  // sentinel end offset
  }

  static int64_t KeyForCoord(const Coordinate& c) {
    return CellKey(static_cast<int32_t>(std::floor(c.latitude)),
                   static_cast<int32_t>(std::floor(c.longitude)));
  }

  // Collect candidate vertices within `radius_deg` cells of the position.
  std::vector<int> Near(const Coordinate& c, int radius_deg) const {
    std::vector<int> out;
    const int32_t lat0 = static_cast<int32_t>(std::floor(c.latitude));
    const int32_t lon0 = static_cast<int32_t>(std::floor(c.longitude));
    for (int dlat = -radius_deg; dlat <= radius_deg; ++dlat) {
      for (int dlon = -radius_deg; dlon <= radius_deg; ++dlon) {
        int32_t lon = lon0 + dlon;
        // Wrap across the antimeridian so a search near lon 179° also sees
        // cells at -180°/-179° (and vice versa). Without this, an airport just
        // east of +180 would miss the nearest fixes just west of -180.
        // Latitude is bounded [-90, 90], so out-of-range lat cells simply have
        // no bucket and miss harmlessly.
        if (lon < -180) lon += 360;
        if (lon >= 180) lon -= 360;
        AppendCell(CellKey(lat0 + dlat, lon), out);
      }
    }
    return out;
  }

 private:
  // Pack a (lat, lon) degree cell into a fixed-width 64-bit key. lat is offset
  // to [0, 180] and lon to [0, 360]; the *1000 spacing keeps lon in the low
  // three digits so the pair is injective across the whole globe. Fixed-width
  // types (not long/int) so the key is identical on every platform.
  static int64_t CellKey(int32_t lat, int32_t lon) {
    // Wrap longitude to [-180, 180) so a fix at exactly +180.0 is stored under
    // the same cell the antimeridian-wrapped search (Near maps lon 180 -> -180)
    // probes. Without this the +180 fix stores under (180+180)=360 while the
    // search reads (-180+180)=0 and never finds it.
    if (lon >= 180) lon -= 360;
    if (lon < -180) lon += 360;
    return static_cast<int64_t>(lat + 90) * 1000 + (lon + 180);
  }

  void AppendCell(int64_t key, std::vector<int>& out) const {
    const auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
    if (it != keys_.end() && *it == key) {
      const int i = static_cast<int>(it - keys_.begin());
      out.insert(out.end(), ids_.begin() + starts_[i], ids_.begin() + starts_[i + 1]);
    }
  }

  std::vector<int64_t> keys_;  // sorted unique cell keys
  std::vector<int> starts_;    // size keys_.size()+1: id range [starts_[i], starts_[i+1])
  std::vector<int> ids_;       // flattened vertex ids, grouped by cell in key order
};

GraphBuilder::GraphBuilder(const NavData& data) {
  const int waypoint_count = static_cast<int>(data.waypoints.size());
  const int airport_count = static_cast<int>(data.airports.size());
  const int total = waypoint_count + airport_count;

  // --- Vertices: waypoints first, then airports. ---
  graph_.coords_.reserve(total);
  idents_.reserve(total);
  kinds_.reserve(total);
  airport_elevations_ft_.reserve(airport_count);
  for (int i = 0; i < waypoint_count; ++i) {
    const Waypoint& w = data.waypoints[i];
    graph_.coords_.push_back(w.coord);
    idents_.push_back(FixedIdent::FromIdent(w.ident));
    kinds_.push_back(w.kind);
  }
  for (int i = 0; i < airport_count; ++i) {
    const Airport& a = data.airports[i];
    graph_.coords_.push_back(a.coord);
    idents_.push_back(FixedIdent::FromParts(a.icao, a.arinc424_icao_code));
    kinds_.push_back(WaypointKind::kFix);  // airports have no navaid kind
    airport_elevations_ft_.push_back(a.elevation_ft);
  }
  first_airport_vertex_ = waypoint_count;
  // Build the sorted lookup indices now: airway resolution below queries them by
  // (ident, region). Both build paths (here and FromSnapshot) go through this.
  RebuildIndices();

  // --- Airway-name table; "DCT" reserved at index 0 for synthetic edges. ---
  airway_names_.push_back(std::string(kDctToken));
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

  // --- On-network flags: per-vertex has_outbound (>=1 outgoing airway edge) and
  // has_inbound (>=1 incoming airway edge), derived from airway edges only. A
  // SID must hand off to an outbound fix; a STAR must be picked up at an inbound
  // fix. A forward-only airway that dead-ends at a fix (e.g. a STAR entry gate)
  // leaves that fix inbound-only. Airport vertices have no airway edges, so they
  // are never on-network; they connect per-route via NearestOnNetwork (the DCT
  // fallback in procedure_connector) rather than via static edges in the CSR.
  // (The previous design seeded each airport with up to N bidirectional DCT
  // edges to nearby fixes, but airport vertices are node_blocked for every
  // search role and endpoints are always seeded connection fixes, so those edges
  // were never traversed -- ~45% of all edges were dead weight. They were
  // removed; NearestOnNetwork now carries that connectivity at query time.)
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

  // --- Flatten adjacency to CSR. ---
  graph_.offsets_.resize(total + 1, 0);
  for (int v = 0; v < total; ++v) {
    graph_.offsets_[v + 1] = graph_.offsets_[v] + static_cast<int>(adj[v].size());
  }
  graph_.edges_.reserve(graph_.offsets_[total]);
  for (int v = 0; v < total; ++v) {
    graph_.edges_.insert(graph_.edges_.end(), adj[v].begin(), adj[v].end());
  }

  // Persist the on-network flags (computed from airway edges only) so procedure
  // wiring can tell true enroute vertices from terminal-only fixes, and can pick
  // the right direction: outbound for SID, inbound for STAR.
  has_outbound_ = std::move(has_outbound);
  has_inbound_ = std::move(has_inbound);

  // Build the resident spatial index over waypoints so NearestOnNetwork (the
  // per-route DCT fallback) is ~O(candidates) instead of an O(V) scan.
  RebuildGrid();
}

GraphBuilder::~GraphBuilder() = default;  // DegreeGrid is complete only in this TU.
GraphBuilder::GraphBuilder(GraphBuilder&&) noexcept = default;

void GraphBuilder::RebuildGrid() {
  // Only waypoints participate (airports are never on-network: has_outbound_/
  // has_inbound_ derive from airway edges only, and airports have none). Index
  // the contiguous waypoint range [0, first_airport_vertex_).
  std::vector<std::pair<int64_t, int>> entries;
  entries.reserve(first_airport_vertex_);
  for (int v = 0; v < first_airport_vertex_; ++v) {
    entries.emplace_back(DegreeGrid::KeyForCoord(graph_.coords_[v]), v);
  }
  grid_ = std::make_unique<DegreeGrid>(std::move(entries));
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
  if (ident.ident.size() > FixedIdent::kIdentCap ||
      ident.arinc424_icao_code.size() > FixedIdent::kArinc424IcaoCodeCap) {
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
  const std::vector<uint8_t>& mask = inbound ? has_inbound_ : has_outbound_;
  // Score each on-network candidate with its distance computed exactly once; a
  // sort comparator would recompute DistanceTo O(log V) times per element. Pairs
  // sort by (distance, vertex), so ties break deterministically on vertex id.
  std::vector<std::pair<double, int>> scored;

  if (grid_) {
    // Expand the cell search radius until enough on-network candidates are
    // found. The geometric progression reaches a whole-globe scan (radius 180)
    // so sparse regions (poles / mid-ocean) still return the global nearest set
    // -- matching the previous O(V) scan exactly, so route output stays
    // bit-identical. In practice radius 2-4 already yields far more than `count`
    // candidates, so the loop exits early and never touches most of the graph.
    // A dedup set is needed because antimeridian wrapping can make a large
    // radius visit the same cell twice.
    std::unordered_set<int> seen;
    for (int radius : {2, 4, 8, 16, 32, 64, 128, 180}) {
      scored.clear();
      seen.clear();
      for (int cand : grid_->Near(coord, radius)) {
        if (mask[cand] && seen.insert(cand).second) {
          scored.emplace_back(coord.DistanceTo(graph_.coords_[cand]), cand);
        }
      }
      if (static_cast<int>(scored.size()) >= count) {
        break;
      }
    }
  } else {
    // Moved-from builder (no grid): full-scan fallback. Does not happen on a
    // live NavDatabase, but keeps the method total.
    scored.reserve(256);
    const int v_count = graph_.VertexCount();
    for (int v = 0; v < v_count; ++v) {
      if (mask[v]) {
        scored.emplace_back(coord.DistanceTo(graph_.coords_[v]), v);
      }
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
  if (ident.size() > FixedName8::kCap) {
    return {};  // longer than any stored ident -> no match (see VertexByIdent)
  }
  const FixedName8 key = FixedName8::From(ident);
  auto lo = std::lower_bound(
      ident_all_.begin(), ident_all_.end(), key,
      [](const std::pair<FixedName8, int>& e, const FixedName8& k) { return e.first < k; });
  std::vector<int> out;
  for (auto it = lo; it != ident_all_.end() && it->first == key; ++it) {
    out.push_back(it->second);
  }
  return out;
}

int GraphBuilder::VertexByAirport(const std::string& icao) const {
  if (icao.size() > FixedName8::kCap) {
    return -1;  // longer than any stored ICAO -> no match (see VertexByIdent)
  }
  const FixedName8 key = FixedName8::From(icao);
  auto it = std::lower_bound(
      airport_index_.begin(), airport_index_.end(), key,
      [](const std::pair<FixedName8, int>& e, const FixedName8& k) { return e.first < k; });
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
    ident_all_.emplace_back(FixedName8::From(idents_[i].IdentView()), i);
  }
  for (int v = first_airport_vertex_; v < v_count; ++v) {
    airport_index_.emplace_back(FixedName8::From(idents_[v].IdentView()), v);
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
  // Rebuild the spatial index from the loaded coords so NearestOnNetwork is
  // indexed on the cache path too (not just the parse path).
  b.RebuildGrid();
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
