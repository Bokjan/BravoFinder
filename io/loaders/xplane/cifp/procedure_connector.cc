#include "io/loaders/xplane/cifp/procedure_connector.h"

#include <algorithm>
#include <unordered_map>

#include "core/domain/coordinate.h"
#include "io/graph_builder.h"

namespace bf {

namespace {

// Accumulate an estimated leg distance. Definite-fix legs use the great-circle
// distance between consecutive resolved fixes; heading/altitude/arc legs fall
// back to the CIFP-provided leg distance (or a small default when absent), per
// the M3 "collapse to an equivalent edge" approach.
double LegDistance(const ProcedureLeg& leg, const Coordinate* prev_coord,
                   const Coordinate* this_coord) {
  if (prev_coord != nullptr && this_coord != nullptr) {
    return prev_coord->DistanceTo(*this_coord);
  }
  if (leg.distance_nm > 0.0) {
    return leg.distance_nm;
  }
  return 0.0;
}

// Resolve a procedure leg's fix to a graph vertex, preferring the full
// (ident, region) key. Returns -1 when the leg has no resolvable fix.
int ResolveFix(const ProcedureLeg& leg, const GraphBuilder& builder) {
  if (leg.fix.ident.empty()) {
    return -1;
  }
  int v = builder.VertexByIdent(leg.fix);
  if (v < 0) {
    v = builder.VertexByIdent(leg.fix.ident);
  }
  return v;
}

// Whether this procedure record should be considered, honoring an optional
// runway filter. An empty filter accepts all; otherwise the procedure must be
// for that runway (or be runway-independent, i.e. carry no runway).
bool RunwayMatches(const Procedure& p, const std::string& runway_filter) {
  if (runway_filter.empty()) {
    return true;
  }
  return p.runway.empty() || p.runway == runway_filter;
}

// Build a ProcedureRef describing one procedure record.
ProcedureRef MakeRef(const Procedure& p) {
  ProcedureRef ref;
  ref.type = p.type;
  ref.name = p.name;
  ref.transition = p.transition_ident;
  ref.runway = p.runway;
  return ref;
}

// Merge a (fix_vertex, seed, ref) finding into the connection map, keeping the
// smallest seed distance per fix and collecting every procedure ref.
void Accumulate(std::unordered_map<int, Connection>& by_fix, int fix_vertex, double seed,
                const ProcedureRef& ref) {
  auto it = by_fix.find(fix_vertex);
  if (it == by_fix.end()) {
    Connection c;
    c.fix_vertex = fix_vertex;
    c.seed_distance_nm = seed;
    c.procedures.push_back(ref);
    by_fix.emplace(fix_vertex, std::move(c));
    return;
  }
  it->second.procedures.push_back(ref);
  if (seed < it->second.seed_distance_nm) {
    it->second.seed_distance_nm = seed;
  }
}

// Walk a procedure's legs accumulating distance, recording each on-network
// definite-fix it passes as (vertex, cumulative distance from the runway end).
struct FixHit {
  int vertex;
  double cumulative_nm;
};

std::vector<FixHit> WalkOnNetworkFixes(const Procedure& p, const GraphBuilder& builder) {
  std::vector<FixHit> hits;
  double cumulative = 0.0;
  bool have_prev = false;
  Coordinate prev_coord{};
  for (const ProcedureLeg& leg : p.legs) {
    int v = leg.fix_is_definite() ? ResolveFix(leg, builder) : -1;
    Coordinate this_coord{};
    bool have_this = false;
    if (v >= 0) {
      this_coord = builder.graph().CoordOf(v);
      have_this = true;
    }
    cumulative +=
        LegDistance(leg, have_prev ? &prev_coord : nullptr, have_this ? &this_coord : nullptr);
    if (v >= 0 && builder.OnNetwork(v)) {
      hits.push_back(FixHit{v, cumulative});
    }
    if (have_this) {
      prev_coord = this_coord;
      have_prev = true;
    }
  }
  return hits;
}

std::vector<Connection> Finalize(std::unordered_map<int, Connection>& by_fix) {
  std::vector<Connection> out;
  out.reserve(by_fix.size());
  for (auto& [vertex, conn] : by_fix) {
    out.push_back(std::move(conn));
  }
  // Stable ordering by seed distance keeps results deterministic.
  std::sort(out.begin(), out.end(), [](const Connection& a, const Connection& b) {
    if (a.seed_distance_nm != b.seed_distance_nm) {
      return a.seed_distance_nm < b.seed_distance_nm;
    }
    return a.fix_vertex < b.fix_vertex;
  });
  return out;
}

}  // namespace

std::vector<Connection> ProcedureConnector::BuildDeparture(const CifpData& cifp,
                                                           const Coordinate& airport_coord,
                                                           const GraphBuilder& builder,
                                                           const std::string& runway_filter) {
  // A SID delivers the aircraft to the network at the LAST on-network fix it
  // reaches. The seed is the straight-line distance from the airport to that
  // fix: an M3 estimate that never undercounts the true track (precise
  // procedure geometry is a later milestone), so total route distances stay
  // physically plausible. WalkOnNetworkFixes is used only to pick the handoff
  // fix, not to measure it, since a published SID spans several CIFP records
  // (runway transition + common segment + enroute transition).
  std::unordered_map<int, Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kSid || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const std::vector<FixHit> hits = WalkOnNetworkFixes(p, builder);
    if (hits.empty()) {
      continue;
    }
    const int fix = hits.back().vertex;
    const double seed = airport_coord.DistanceTo(builder.graph().CoordOf(fix));
    Accumulate(by_fix, fix, seed, MakeRef(p));
  }
  return Finalize(by_fix);
}

std::vector<Connection> ProcedureConnector::BuildArrival(const CifpData& cifp,
                                                         const Coordinate& airport_coord,
                                                         const GraphBuilder& builder,
                                                         const std::string& runway_filter) {
  // A STAR picks the aircraft up at the FIRST on-network fix. As for departures,
  // the seed is the straight-line distance from that fix to the airport (an
  // estimate that never undercounts), since a published STAR also spans several
  // CIFP records.
  std::unordered_map<int, Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kStar || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const std::vector<FixHit> hits = WalkOnNetworkFixes(p, builder);
    if (hits.empty()) {
      continue;
    }
    const int fix = hits.front().vertex;
    const double seed = airport_coord.DistanceTo(builder.graph().CoordOf(fix));
    Accumulate(by_fix, fix, seed, MakeRef(p));
  }
  return Finalize(by_fix);
}

std::vector<Connection> ProcedureConnector::BuildDctFallback(const Coordinate& airport_coord,
                                                             const GraphBuilder& builder,
                                                             int count) {
  std::vector<Connection> out;
  for (int v : builder.NearestOnNetwork(airport_coord, count)) {
    Connection c;
    c.fix_vertex = v;
    c.seed_distance_nm = airport_coord.DistanceTo(builder.graph().CoordOf(v));
    out.push_back(std::move(c));  // no ProcedureRef: this is a DCT connection
  }
  return out;
}

std::vector<SeededEndpoint> ProcedureConnector::ToEndpoints(
    const std::vector<Connection>& connections) {
  std::vector<SeededEndpoint> endpoints;
  endpoints.reserve(connections.size());
  for (const Connection& c : connections) {
    endpoints.push_back(SeededEndpoint{c.fix_vertex, c.seed_distance_nm});
  }
  return endpoints;
}

}  // namespace bf
