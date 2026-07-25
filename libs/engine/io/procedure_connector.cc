// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/procedure_connector.h"

#include <algorithm>
#include <unordered_map>

#include "core/domain/coordinate.h"
#include "io/graph_builder.h"

namespace bf {

namespace {

// Accumulate an estimated leg distance. Definite-fix legs use the great-circle
// distance between consecutive resolved fixes; heading/altitude/arc legs, which
// have no resolvable end fix, fall back to the CIFP-provided leg distance (0.0
// when the leg carries none).
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

// Resolve a procedure leg's fix to a graph vertex by its full (ident, region)
// key. Returns -1 when the leg has no resolvable fix. The ident-only fallback
// was removed: a procedure leg carries its region, so resolving by
// (ident, region) is both correct and unambiguous, and silently guessing a
// region would risk wiring a procedure to the wrong fix.
int ResolveFix(const ProcedureLeg& leg, const GraphBuilder& builder) {
  if (leg.fix.IdentView().empty()) {
    return -1;
  }
  return builder.VertexByIdent(leg.fix);
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

// One on-network fix a procedure record passes, with the polyline distance from
// the record's start accumulated up to it.
struct FixHit {
  int vertex;
  double cumulative_nm;
};

// The result of walking one procedure record's legs: every on-network fix it
// reaches, the total polyline length of the record, and the first/last resolved
// fix coordinates (used to bridge the record's endpoints to the airport with a
// straight line, since the runway-to-first-fix and last-fix-to-runway portions
// are not measured here).
struct WalkResult {
  std::vector<FixHit> hits;
  double total_nm = 0.0;
  Coordinate first_coord{};
  Coordinate last_coord{};
  bool have_first = false;
  bool have_last = false;
};

// Which airway-edge direction makes a fix a usable procedure connection point.
// A SID hands the aircraft to a fix it then departs along an airway (needs an
// outbound edge); a STAR picks the aircraft up at a fix reached along an airway
// (needs an inbound edge). A forward-only airway that dead-ends at a STAR entry
// gate leaves that fix inbound-only, so the two sides must not share one test.
enum class WalkDir { kOutbound, kInbound };

WalkResult WalkOnNetworkFixes(const Procedure& p, const GraphBuilder& builder, WalkDir dir) {
  WalkResult result;
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
    if (v >= 0) {
      const bool usable = dir == WalkDir::kInbound ? builder.HasInbound(v) : builder.HasOutbound(v);
      if (usable) {
        result.hits.push_back(FixHit{v, cumulative});
      }
    }
    if (have_this) {
      if (!result.have_first) {
        result.first_coord = this_coord;
        result.have_first = true;
      }
      result.last_coord = this_coord;
      result.have_last = true;
      prev_coord = this_coord;
      have_prev = true;
    }
  }
  result.total_nm = cumulative;
  return result;
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
  // A SID can hand the aircraft to the network at ANY on-network fix it passes,
  // not just its last one: filing "join the airway at <fix>" is routine. Every
  // such fix is exposed as a candidate connection and the multi-source search
  // picks whichever minimizes seed + enroute cost.
  //
  // The seed is the estimated distance flown from the runway to that fix: the
  // straight line from the airport to the record's first resolved fix (the
  // unmeasured runway-to-first-fix portion) plus the record's own polyline up to
  // the candidate. The polyline follows the published track, so a fix reached
  // only after a long detour gets a larger (more honest) seed than its straight
  // -line distance would suggest, steering the search toward closer fixes.
  std::unordered_map<int, Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kSid || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const WalkResult walk = WalkOnNetworkFixes(p, builder, WalkDir::kOutbound);
    if (walk.hits.empty()) {
      continue;
    }
    const double runway_bridge = walk.have_first ? airport_coord.DistanceTo(walk.first_coord) : 0.0;
    for (const FixHit& hit : walk.hits) {
      const double seed = runway_bridge + hit.cumulative_nm;
      Accumulate(by_fix, hit.vertex, seed, MakeRef(p));
    }
  }
  return Finalize(by_fix);
}

std::vector<Connection> ProcedureConnector::BuildArrival(const CifpData& cifp,
                                                         const Coordinate& airport_coord,
                                                         const GraphBuilder& builder,
                                                         const std::string& runway_filter) {
  // A STAR can pick the aircraft up at ANY on-network fix it passes, not just
  // its first one. Every such fix is exposed as a candidate; the multi-source
  // search picks the cheapest entry. The seed is the estimated distance from
  // that fix to the runway: the record's polyline from the candidate to the last
  // resolved fix, plus the straight line from there to the airport (the
  // unmeasured last-fix-to-runway portion). Following the published track makes
  // a far entry fix (e.g. KLAX BASET5 via PGS, ~260 NM out) cost more than a
  // nearer one on the same STAR (e.g. CIVET), so the search prefers the latter.
  std::unordered_map<int, Connection> by_fix;
  for (const Procedure& p : cifp.procedures) {
    if (p.type != ProcedureType::kStar || !RunwayMatches(p, runway_filter)) {
      continue;
    }
    const WalkResult walk = WalkOnNetworkFixes(p, builder, WalkDir::kInbound);
    if (walk.hits.empty()) {
      continue;
    }
    const double runway_bridge = walk.have_last ? walk.last_coord.DistanceTo(airport_coord) : 0.0;
    for (const FixHit& hit : walk.hits) {
      const double seed = (walk.total_nm - hit.cumulative_nm) + runway_bridge;
      Accumulate(by_fix, hit.vertex, seed, MakeRef(p));
    }
  }
  return Finalize(by_fix);
}

std::vector<Connection> ProcedureConnector::BuildDctFallback(const Coordinate& airport_coord,
                                                             const GraphBuilder& builder, int count,
                                                             bool arrival) {
  std::vector<Connection> out;
  for (int v : builder.NearestOnNetwork(airport_coord, count, /*inbound=*/arrival)) {
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
