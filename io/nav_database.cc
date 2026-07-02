#include "io/nav_database.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/constraints/altitude_constraints.h"
#include "core/constraints/avoid_constraint.h"
#include "core/constraints/mora_constraint.h"
#include "core/constraints/randomize_constraint.h"
#include "core/graph/yen_kshortest.h"
#include "core/routing/route_parser.h"
#include "core/routing/route_string.h"
#include "core/version.h"
#include "io/cache/bfdb_cache.h"
#include "io/cache/cifp_cache.h"
#include "io/graph_builder.h"
#include "io/loaders/xplane/cifp/cifp_parser.h"
#include "io/loaders/xplane/cifp/procedure_connector.h"
#include "io/loaders/xplane/xplane_loader.h"

namespace bf {

namespace {

std::string ToUpper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); });
  return s;
}

// How one endpoint of a query attaches to the enroute graph. An airport with
// procedures contributes several seeded connection fixes; a plain waypoint or a
// DCT-fallback airport contributes one or a few. `airport_icao` is empty for a
// bare waypoint endpoint. `has_procedures` records whether the airport actually
// publishes procedures for this side (SID for departure, STAR for arrival), so a
// DCT fallback can be told apart from missing data: procedures that exist but
// reach no on-network fix (radar vectors) still fall back to DCT.
struct EndpointPlan {
  std::vector<Connection> connections;
  std::string airport_icao;  // empty if the endpoint is a plain waypoint
  bool used_procedures = false;
  bool has_procedures = false;
  // Set when the request named a SID/STAR that the airport does not publish (or
  // whose fixes reach no on-network vertex): the caller reports an Error instead
  // of silently falling back to DCT or another procedure.
  bool named_procedure_unmatched = false;
};

// Format a procedure reference as "NAME.TRANSITION" (or just "NAME" when the
// transition is empty / the common segment).
std::string FormatRef(const ProcedureRef& ref) {
  if (ref.transition.empty()) {
    return ref.name;
  }
  return ref.name + "." + ref.transition;
}

// Whether a procedure ref matches a requested selector. The selector is either
// a bare name ("DEEZZ5", matches any transition) or "NAME.TRANSITION"
// ("DEEZZ5.TOWIN", matches that transition exactly). Comparison is
// case-sensitive (CIFP names are already upper-case).
bool RefMatchesSelector(const ProcedureRef& ref, const std::string& selector) {
  const size_t dot = selector.find('.');
  if (dot == std::string::npos) {
    return ref.name == selector;
  }
  return ref.name == selector.substr(0, dot) && ref.transition == selector.substr(dot + 1);
}

// Filter connections in place to only those procedures matching `selector`,
// dropping any connection left with no matching procedure. Returns true if at
// least one procedure survived. A no-op returning true when the selector is
// empty (no name requested).
bool FilterConnectionsByName(std::vector<Connection>& connections, const std::string& selector) {
  if (selector.empty()) {
    return true;
  }
  bool any = false;
  for (Connection& c : connections) {
    std::vector<ProcedureRef> kept;
    for (const ProcedureRef& ref : c.procedures) {
      if (RefMatchesSelector(ref, selector)) {
        kept.push_back(ref);
      }
    }
    c.procedures = std::move(kept);
    if (!c.procedures.empty()) {
      any = true;
    }
  }
  if (any) {
    // Drop connections that no longer carry any matching procedure so the search
    // only seeds fixes reachable by the requested procedure.
    std::vector<Connection> filtered;
    for (Connection& c : connections) {
      if (!c.procedures.empty()) {
        filtered.push_back(std::move(c));
      }
    }
    connections = std::move(filtered);
  }
  return any;
}

// Build a Route from a path of connection-fix vertices. `dep_label`/`arr_label`
// are the airport ICAOs to show as the true endpoints (empty for waypoint
// endpoints). When an airport endpoint connects through a procedure, `sid`/
// `star` name it and `dep_seed`/`arr_seed` are the estimated procedure
// distances; these become explicit first/last legs (airport <-> connection fix)
// and are embedded in the route string like a filed flight plan.
Route MakeRoute(const GraphBuilder& builder, const NavGraph& graph, const ShortestPath& path,
                const std::string& dep_label, const std::string& arr_label, const std::string& sid,
                const std::string& star, double dep_seed, double arr_seed) {
  Route route;
  route.total_distance_nm = path.distance_nm;
  if (path.vertices.empty()) {
    return route;
  }
  const std::string dep_fix_id = builder.IdentOf(path.vertices.front()).ident;
  const std::string arr_fix_id = builder.IdentOf(path.vertices.back()).ident;

  // Points: optional departure airport, the connection fixes along the path,
  // then the optional arrival airport.
  if (!dep_label.empty()) {
    route.points.push_back(RoutePoint{dep_label, graph.CoordOf(path.vertices.front())});
  }
  for (int v : path.vertices) {
    route.points.push_back(RoutePoint{builder.IdentOf(v).ident, graph.CoordOf(v)});
  }
  if (!arr_label.empty()) {
    route.points.push_back(RoutePoint{arr_label, graph.CoordOf(path.vertices.back())});
  }

  // Leading procedure leg: airport -> first connection fix via the SID (or DCT
  // when the airport fell back to a direct link).
  if (!dep_label.empty()) {
    route.legs.push_back(RouteLeg{dep_label, dep_fix_id, sid.empty() ? "DCT" : sid, dep_seed, {}});
  }
  // Enroute legs between consecutive on-network fixes.
  for (size_t i = 0; i + 1 < path.vertices.size(); ++i) {
    const int u = path.vertices[i];
    const int w = path.vertices[i + 1];
    std::string via = "DCT";
    double dist = 0.0;
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->to == w) {
        via = builder.AirwayName(e->airway_id);
        dist = e->distance_nm;
        break;
      }
    }
    route.legs.push_back(
        RouteLeg{builder.IdentOf(u).ident, builder.IdentOf(w).ident, via, dist, {}});
  }
  // Trailing procedure leg: last connection fix -> airport via the STAR.
  if (!arr_label.empty()) {
    route.legs.push_back(
        RouteLeg{arr_fix_id, arr_label, star.empty() ? "DCT" : star, arr_seed, {}});
  }

  // Route string in filed-flight-plan style: DEP SID FIX <airways> FIX STAR ARR.
  // BuildRouteString folds consecutive legs on a shared airway (listing it only
  // at the join/leave fixes) and, as a side effect, rewrites each leg's `via` to
  // the single chosen designator and records any concurrency in
  // `concurrent_airways`.
  const std::string first_point = route.points.empty() ? "" : route.points.front().ident;
  route.route_string = BuildRouteString(first_point, route.legs);
  return route;
}

// Pick the primary procedure (name + runway) and all interchangeable options
// for the connection fix `fix_vertex` within `plan`. Returns the chosen name in
// `name`/`runway` and every "NAME.TRANSITION" sharing the fix in `options`.
void SelectProcedures(const EndpointPlan& plan, int fix_vertex, std::string& name,
                      std::string& runway, std::vector<std::string>& options) {
  for (const Connection& c : plan.connections) {
    if (c.fix_vertex != fix_vertex) {
      continue;
    }
    for (const ProcedureRef& ref : c.procedures) {
      options.push_back(FormatRef(ref));
      if (name.empty()) {
        name = ref.name;
        runway = ref.runway;
      }
    }
    break;
  }
}

// Resolve the request's avoid_waypoints to the set of vertices to block. A full
// "IDENT/REGION" key resolves to that single vertex; a bare "IDENT" resolves to
// every region's match (idents are not globally unique, so "avoid X" avoids all
// X). Unknown idents contribute nothing (avoiding something absent is a no-op).
std::unordered_set<int> ResolveAvoidVertices(const GraphBuilder& builder,
                                             const std::vector<std::string>& avoid_waypoints) {
  std::unordered_set<int> out;
  for (const std::string& raw : avoid_waypoints) {
    const std::string up = ToUpper(raw);
    const size_t slash = up.find('/');
    if (slash != std::string::npos) {
      const int v = builder.VertexByIdent(Ident(up.substr(0, slash), up.substr(slash + 1)));
      if (v >= 0) {
        out.insert(v);
      }
    } else {
      for (const int v : builder.VerticesByIdent(up)) {
        out.insert(v);
      }
    }
  }
  return out;
}

// Resolve the request's avoid_airways (by designator) to the set of airway_ids
// to block. Because a stored airway name may be a concurrency ("J60-V123"), an
// airway_id is included when any of its designators is in the avoid set -- so
// avoiding "J60" also blocks segments recorded under "J60-V123".
std::unordered_set<uint16_t> ResolveAvoidAirwayIds(const GraphBuilder& builder,
                                                   const std::vector<std::string>& avoid_airways) {
  std::unordered_set<std::string> wanted;
  for (const std::string& a : avoid_airways) {
    wanted.insert(ToUpper(a));
  }
  std::unordered_set<uint16_t> out;
  if (wanted.empty()) {
    return out;
  }
  const std::vector<std::string>& names = builder.AirwayNames();
  for (size_t id = 1; id < names.size(); ++id) {  // id 0 = "DCT", never avoided
    for (const std::string& designator : SplitDesignators(names[id])) {
      if (wanted.count(designator) != 0) {
        out.insert(static_cast<uint16_t>(id));
        break;
      }
    }
  }
  return out;
}

// Resolve one forced ("via") point token to a graph vertex. A full
// "IDENT/REGION" key resolves exactly; a bare ident with several regional
// matches picks the one adding the least detour to the dep->arr great circle
// (deterministic and explainable). Airports are rejected (a via point is an
// enroute fix, and airports are barred as transit nodes anyway). On success,
// writes the resolved "IDENT/REGION" to `echo`. Returns the vertex, or -1 if no
// non-airport match exists (the caller reports an unknown-forced-point error).
int ResolveForcedPoint(const GraphBuilder& builder, const std::string& token,
                       const Coordinate& from, const Coordinate& to, std::string& echo,
                       bool& is_airport) {
  is_airport = false;
  const std::string up = ToUpper(token);
  const size_t slash = up.find('/');
  if (slash != std::string::npos) {
    const Ident id(up.substr(0, slash), up.substr(slash + 1));
    const int v = builder.VertexByIdent(id);
    if (v < 0) {
      return -1;
    }
    if (builder.IsAirport(v)) {
      is_airport = true;
      return -1;
    }
    echo = id.ident + "/" + id.region;
    return v;
  }
  // Bare ident: choose the non-airport match minimizing the added detour
  // d(from,v) + d(v,to) - d(from,to). Ties break on the lowest vertex index for
  // determinism.
  int best = -1;
  double best_detour = 0.0;
  bool saw_airport = false;
  for (const int v : builder.VerticesByIdent(up)) {
    if (builder.IsAirport(v)) {
      saw_airport = true;
      continue;
    }
    const Coordinate c = builder.graph().CoordOf(v);
    const double detour = from.DistanceTo(c) + c.DistanceTo(to);
    if (best < 0 || detour < best_detour) {
      best = v;
      best_detour = detour;
    }
  }
  if (best < 0) {
    is_airport = saw_airport;  // only matches were airports
    return -1;
  }
  echo = builder.IdentOf(best).ident + "/" + builder.IdentOf(best).region;
  return best;
}

// Stitch a route through an ordered list of forced ("via") vertices. The route
// is searched in hops -- sources -> F1, Fi -> Fi+1 for each interior pair, then
// Fn -> goals -- and concatenated. Only the first hop carries the real source
// seeds and only the last the real goal seeds; interior forced vertices are
// seeded at 0 so their cost is not double counted at the seams. Every hop
// honors all constraints and node/edge bans in `options`.
//
// Up to `k` whole routes are returned, ordered by total (segment-sum) cost.
// Each hop is expanded into up to `k` alternatives via K-shortest; the best K
// end-to-end combinations are then selected by a "sum of per-segment costs"
// best-first merge over the Cartesian product (a lazy K-way merge that touches
// O(k * hops) combinations, not the full product). A combination whose stitched
// path repeats a vertex (a cycle at some seam) is skipped -- forced routing is
// order-sensitive, so a repeated fix is not a valid simple route.
//
// A returned path's distance_nm/cost include both endpoint seeds, matching the
// non-forced path so downstream MakeRoute treats them identically.
std::vector<ShortestPath> FindForcedPaths(const NavGraph& graph,
                                          const std::vector<SeededEndpoint>& sources,
                                          const std::vector<SeededEndpoint>& goals,
                                          const std::vector<int>& forced, int k,
                                          const SearchOptions& options) {
  std::vector<ShortestPath> results;
  if (k <= 0 || forced.empty()) {
    return results;
  }

  // Build each hop's endpoint sets, then its up-to-k candidate paths.
  const size_t hops = forced.size() + 1;
  std::vector<std::vector<ShortestPath>> segments;
  segments.reserve(hops);
  for (size_t h = 0; h < hops; ++h) {
    const std::vector<SeededEndpoint> hop_sources =
        (h == 0) ? sources : std::vector<SeededEndpoint>{SeededEndpoint{forced[h - 1], 0.0}};
    const std::vector<SeededEndpoint> hop_goals =
        (h + 1 == hops) ? goals : std::vector<SeededEndpoint>{SeededEndpoint{forced[h], 0.0}};
    std::vector<ShortestPath> cands = FindKShortestPathsMulti(graph, hop_sources, hop_goals, k, options);
    if (cands.empty()) {
      return results;  // a hop is unroutable -> no forced route exists
    }
    segments.push_back(std::move(cands));
  }

  // Stitch one combination (one candidate index per segment) into a full path.
  // Returns found=false if the segments do not meet or the result has a cycle.
  auto stitch = [&](const std::vector<int>& pick) -> ShortestPath {
    ShortestPath out;
    std::vector<int> path;
    double dist = 0.0;
    double cost = 0.0;
    for (size_t h = 0; h < hops; ++h) {
      const ShortestPath& seg = segments[h][pick[h]];
      if (seg.vertices.empty()) {
        return out;
      }
      if (path.empty()) {
        path = seg.vertices;
      } else {
        if (path.back() != seg.vertices.front()) {
          return out;  // seam mismatch (should not happen: seam == forced fix)
        }
        path.insert(path.end(), seg.vertices.begin() + 1, seg.vertices.end());
      }
      dist += seg.distance_nm;
      cost += seg.cost;
    }
    std::unordered_set<int> seen;
    seen.reserve(path.size());
    for (const int v : path) {
      if (!seen.insert(v).second) {
        return out;  // cycle at a seam -> not a simple route
      }
    }
    out.vertices = std::move(path);
    out.distance_nm = dist;
    out.cost = cost;
    out.found = true;
    return out;
  };

  // Lazy K-way merge over the Cartesian product of segment candidates, ordered
  // by the sum of per-segment costs. Start from the all-best pick and expand a
  // neighbor per segment (increment one index) each time a pick is popped.
  auto combo_cost = [&](const std::vector<int>& pick) {
    double c = 0.0;
    for (size_t h = 0; h < hops; ++h) {
      c += segments[h][pick[h]].cost;
    }
    return c;
  };
  struct HeapItem {
    double cost;
    std::vector<int> pick;
    bool operator>(const HeapItem& o) const { return cost > o.cost; }
  };
  std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<>> heap;
  std::set<std::vector<int>> queued;

  std::vector<int> start(hops, 0);
  heap.push({combo_cost(start), start});
  queued.insert(start);

  while (!heap.empty() && static_cast<int>(results.size()) < k) {
    const std::vector<int> pick = heap.top().pick;
    heap.pop();

    const ShortestPath stitched = stitch(pick);
    if (stitched.found) {
      results.push_back(stitched);
    }

    // Enqueue the neighbors that advance one segment's candidate index.
    for (size_t h = 0; h < hops; ++h) {
      if (pick[h] + 1 < static_cast<int>(segments[h].size())) {
        std::vector<int> next = pick;
        next[h] += 1;
        if (queued.insert(next).second) {
          heap.push({combo_cost(next), next});
        }
      }
    }
  }

  return results;
}

}  // namespace

NavDatabase::NavDatabase() : cache_mutex_(std::make_unique<std::mutex>()) {}
NavDatabase::~NavDatabase() = default;
NavDatabase::NavDatabase(NavDatabase&&) noexcept = default;
NavDatabase& NavDatabase::operator=(NavDatabase&&) noexcept = default;

Result<NavDatabase> NavDatabase::Open(const std::string& data_dir) {
  Result<NavData> data = XPlaneLoader::Load(data_dir);
  if (!data) {
    return Result<NavDatabase>::Err(std::move(data).error());
  }
  NavDatabase db;
  db.data_dir_ = data_dir;
  db.cycle_ = data.value().cycle;
  db.build_ = data.value().build;
  db.mora_ = std::move(data.value().mora);
  db.msa_ = std::move(data.value().msa);
  db.builder_ = std::make_unique<GraphBuilder>(data.value());
  db.BuildAirwayIndex();
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<NavDatabase> NavDatabase::OpenCached(const std::string& bfdb_path,
                                            const std::string& data_dir,
                                            const std::string& cifp_db_path, CifpLoad cifp_load) {
  Result<BfdbImage> image = BfdbCache::Read(bfdb_path);
  if (!image) {
    return Result<NavDatabase>::Err(std::move(image).error());
  }
  BfdbImage& img = image.value();
  NavDatabase db;
  // The CIFP directory: an explicit override wins, else the build-time dir.
  db.data_dir_ = data_dir.empty() ? img.data_dir : data_dir;
  db.cycle_ = img.cycle;
  db.build_ = img.build;
  db.mora_ = std::move(img.mora);
  db.msa_ = std::move(img.msa);
  db.builder_ = std::make_unique<GraphBuilder>(GraphBuilder::FromImage(std::move(img)));

  // Resolve the CIFP procedure cache: an explicit path wins; otherwise look for
  // a sibling "<stem>_cifp.bfdb" next to the graph cache. Either being absent is
  // fine -- ProceduresFor then falls back to CIFP/<ICAO>.dat files.
  std::string cifp_path = cifp_db_path;
  if (cifp_path.empty()) {
    std::filesystem::path p(bfdb_path);
    const std::string stem = p.stem().string();
    cifp_path = (p.parent_path() / (stem + "_cifp.bfdb")).string();
    if (!std::filesystem::exists(cifp_path)) {
      cifp_path.clear();
    }
  }
  if (!cifp_path.empty()) {
    Result<CifpArchive> archive = CifpCache::Open(cifp_path);
    if (!archive) {
      return Result<NavDatabase>::Err(std::move(archive).error());
    }
    if (cifp_load == CifpLoad::kEager) {
      // Deserialize every airport up front into the procedure cache, then freeze
      // it: subsequent ProceduresFor calls only read existing entries, so they
      // need no lock (contract B holds with no shared mutable state).
      std::unordered_map<std::string, CifpData> all = archive.value().FetchAll();
      db.procedure_cache_.reserve(all.size());
      for (auto& entry : all) {
        db.procedure_cache_.emplace(entry.first,
                                    std::make_unique<CifpData>(std::move(entry.second)));
      }
      db.cifp_eager_ = true;
      // The archive is not retained: everything is already in the cache.
    } else {
      db.cifp_archive_ = std::move(archive).value();
    }
  }
  db.BuildAirwayIndex();
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<void> NavDatabase::WriteCache(const std::string& out_path) const {
  if (!builder_) {
    return Result<void>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }
  BfdbImage image = builder_->ToImage(data_dir_, cycle_, build_);
  image.program_semver = kBravoFinderVersion;
  image.source_loader = "xplane";  // the only loader today; recorded as provenance
  image.mora = mora_;
  image.msa = msa_;
  return BfdbCache::Write(out_path, image);
}

Result<uint32_t> NavDatabase::WriteCifpCache(const std::string& out_path,
                                             const std::string& source_loader) const {
  return CifpCache::Build(data_dir_, out_path, source_loader, cycle_, build_, kBravoFinderVersion);
}

const CifpData* NavDatabase::ProceduresFor(const std::string& icao) const {
  // Eager mode: the cache was fully populated at Open and is now frozen, so a
  // plain read needs no lock (no concurrent insert can rehash it). A miss means
  // the airport simply has no procedures.
  if (cifp_eager_) {
    auto it = procedure_cache_.find(icao);
    return it != procedure_cache_.end() ? it->second.get() : nullptr;
  }
  // Fast path: return a cached result (including a cached "no procedures"
  // nullptr) under a brief lock.
  {
    std::lock_guard<std::mutex> guard(*cache_mutex_);
    auto it = procedure_cache_.find(icao);
    if (it != procedure_cache_.end()) {
      return it->second.get();
    }
  }
  // Parse outside the lock so concurrent queries for different airports do not
  // serialize on disk I/O. Two threads racing on the same airport will both
  // parse (harmless, redundant work). Source: the CIFP cache archive if one is
  // loaded (an independent ifstream per fetch, contract-B safe), else the
  // CIFP/<ICAO>.dat file.
  std::unique_ptr<CifpData> stored;
  if (cifp_archive_.has_value()) {
    std::optional<CifpData> fetched = cifp_archive_->Fetch(icao);
    if (fetched.has_value()) {
      stored = std::make_unique<CifpData>(std::move(fetched).value());
    }
  } else {
    Result<CifpData> parsed = CifpParser::Parse(data_dir_ + "/CIFP/" + icao + ".dat");
    if (parsed) {
      stored = std::make_unique<CifpData>(std::move(parsed).value());
    }
  }
  // Re-lock and insert. try_emplace keeps the first inserted value if another
  // thread won the race, so a previously returned pointer is never invalidated;
  // the losing thread's parsed copy is simply discarded. Return the value that
  // actually lives in the cache.
  std::lock_guard<std::mutex> guard(*cache_mutex_);
  auto it = procedure_cache_.try_emplace(icao, std::move(stored)).first;
  return it->second.get();
}

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  // Resolve an endpoint into how it attaches to the network. An airport with
  // CIFP procedures connects through them (procedure-first); without CIFP it
  // falls back to DCT links to the nearest on-network waypoints (M1 behavior);
  // a plain waypoint connects as itself.
  auto plan_endpoint = [&](const std::string& name, bool departure) -> EndpointPlan {
    const std::string up = ToUpper(name);
    EndpointPlan plan;
    const int airport = builder_->VertexByAirport(up);
    if (airport >= 0) {
      plan.airport_icao = up;
      const Coordinate apt = builder_->graph().CoordOf(airport);
      const CifpData* cifp = ProceduresFor(up);
      if (cifp != nullptr) {
        const ProcedureType want = departure ? ProcedureType::kSid : ProcedureType::kStar;
        for (const Procedure& p : cifp->procedures) {
          if (p.type == want) {
            plan.has_procedures = true;
            break;
          }
        }
        const std::string& rwy = departure ? request.departure_runway : request.arrival_runway;
        plan.connections = departure
                               ? ProcedureConnector::BuildDeparture(*cifp, apt, *builder_, rwy)
                               : ProcedureConnector::BuildArrival(*cifp, apt, *builder_, rwy);
        // Optional SID/STAR selection by name: keep only the requested procedure.
        // If none matches, mark it so the caller errors instead of falling back.
        const std::string& sel = departure ? request.departure_sid : request.arrival_star;
        if (!sel.empty() && !FilterConnectionsByName(plan.connections, sel)) {
          plan.connections.clear();
          plan.named_procedure_unmatched = true;
          return plan;
        }
        plan.used_procedures = !plan.connections.empty();
      } else if (!(departure ? request.departure_sid : request.arrival_star).empty()) {
        // A procedure was named but the airport has no CIFP data at all.
        plan.named_procedure_unmatched = true;
        return plan;
      }
      if (plan.connections.empty()) {
        // No usable procedures: fall back to DCT links to the nearest
        // on-network waypoints. The airport stays the route endpoint; the
        // connecting leg shows "DCT" since no procedure was selected.
        plan.connections = ProcedureConnector::BuildDctFallback(apt, *builder_, 5);
      }
      return plan;
    }
    // A bare ident is no longer accepted as a route endpoint: idents are not
    // globally unique, and silently picking one region's match would put the
    // whole route on the wrong endpoint. The caller must use an airport ICAO or
    // a (ident, region) pair. With no airport and no ident hit, plan.connections
    // stays empty, so the caller reports an "unknown endpoint" error.
    return plan;
  };

  EndpointPlan dep = plan_endpoint(request.departure, /*departure=*/true);
  if (dep.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "departure airport " + request.departure +
                                                              " has no SID matching '" +
                                                              request.departure_sid + "'"));
  }
  if (dep.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown departure: " + request.departure));
  }
  EndpointPlan arr = plan_endpoint(request.arrival, /*departure=*/false);
  if (arr.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "arrival airport " + request.arrival +
                                                              " has no STAR matching '" +
                                                              request.arrival_star + "'"));
  }
  if (arr.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown arrival: " + request.arrival));
  }

  // Assemble the active constraints from the request.
  AltitudeBandConstraint altitude_band;
  MoraConstraint mora(mora_);
  LevelPreferenceConstraint level_pref;
  // Resolve avoid sets once; the constraint holds them for the whole search
  // (and every Yen spur), so it must outlive the calls below. The vertex set is
  // also used to prune seeded endpoints (below): AvoidConstraint only blocks
  // edges entering a vertex, but a source/goal fix is seeded, not entered, so an
  // avoided connection fix must be removed from the endpoint sets directly.
  const std::unordered_set<int> avoid_vertices =
      ResolveAvoidVertices(*builder_, request.avoid_waypoints);
  AvoidConstraint avoid(avoid_vertices, ResolveAvoidAirwayIds(*builder_, request.avoid_airways));
  RandomizeConstraint randomize(request.random_seed.value_or(0));
  SearchOptions options;
  options.request = &request;
  if (request.altitude.has_value()) {
    options.constraints.push_back(&altitude_band);
    options.constraints.push_back(&mora);
  }
  if (request.level != LevelPreference::kNone) {
    options.constraints.push_back(&level_pref);
  }
  if (!request.avoid_waypoints.empty() || !request.avoid_airways.empty()) {
    options.constraints.push_back(&avoid);
  }
  if (request.random_seed.has_value()) {
    options.constraints.push_back(&randomize);
  }
  // Airports must not be transit nodes: their synthetic DCT links would let the
  // search cut through an unrelated airport (e.g. ...MIE DCT KMIE SNKPT...).
  // Endpoints connect via seeded connection fixes, not airport vertices, so
  // blocking all airport vertices as intermediate nodes is safe.
  const GraphBuilder* builder_ptr = builder_.get();
  options.node_blocked = [builder_ptr](int v) { return builder_ptr->IsAirport(v); };

  const NavGraph& graph = builder_->graph();
  std::vector<SeededEndpoint> sources = ProcedureConnector::ToEndpoints(dep.connections);
  std::vector<SeededEndpoint> goals = ProcedureConnector::ToEndpoints(arr.connections);
  // Drop any seeded connection fix the request asks to avoid: it would otherwise
  // slip through as a search start/end, which AvoidConstraint cannot catch.
  if (!avoid_vertices.empty()) {
    auto drop_avoided = [&](std::vector<SeededEndpoint>& eps) {
      eps.erase(std::remove_if(eps.begin(), eps.end(),
                               [&](const SeededEndpoint& e) {
                                 return avoid_vertices.count(e.vertex) != 0;
                               }),
                eps.end());
    };
    drop_avoided(sources);
    drop_avoided(goals);
    if (sources.empty() || goals.empty()) {
      return Result<Routes>::Err(
          Error(ErrorCode::kNoRoute, "no route between endpoints (avoided all connection fixes)"));
    }
  }

  // Resolve forced ("via") points to an ordered vertex list. Disambiguation of a
  // bare ident uses the dep->arr great circle: pick the match adding the least
  // detour. Endpoint coordinates come from the airport vertex when there is one,
  // else from the first seeded connection fix.
  std::vector<int> forced;
  std::vector<std::string> forced_echo;
  if (!request.forced_points.empty()) {
    const int dep_apt = builder_->VertexByAirport(ToUpper(request.departure));
    const int arr_apt = builder_->VertexByAirport(ToUpper(request.arrival));
    const Coordinate dep_coord =
        dep_apt >= 0 ? graph.CoordOf(dep_apt) : graph.CoordOf(sources.front().vertex);
    const Coordinate arr_coord =
        arr_apt >= 0 ? graph.CoordOf(arr_apt) : graph.CoordOf(goals.front().vertex);
    forced.reserve(request.forced_points.size());
    forced_echo.reserve(request.forced_points.size());
    for (const std::string& token : request.forced_points) {
      std::string echo;
      bool is_airport = false;
      const int v =
          ResolveForcedPoint(*builder_, token, dep_coord, arr_coord, echo, is_airport);
      if (v < 0) {
        const std::string why =
            is_airport ? "' is an airport, not an enroute waypoint" : "' is not a known waypoint";
        return Result<Routes>::Err(
            Error(ErrorCode::kNoRoute, "forced point '" + token + why));
      }
      if (avoid_vertices.count(v) != 0) {
        return Result<Routes>::Err(
            Error(ErrorCode::kNoRoute, "forced point '" + token + "' is also in the avoid list"));
      }
      forced.push_back(v);
      forced_echo.push_back(echo);
    }
  }

  // Find up to k candidate routes. Unlike the earlier scheme that fixed a single
  // best connection-fix pair and only varied the enroute portion between them,
  // the multi-endpoint Yen lets each candidate join through a different SID/STAR
  // connection fix, so the alternatives can use genuinely different procedures.
  // Both forms report distance_nm with both seed costs already included.
  const int k = std::max(1, request.k);
  std::vector<ShortestPath> paths;
  if (!forced.empty()) {
    // Forced points: search each hop (sources -> F1 -> ... -> Fn -> goals) with
    // K-shortest and merge the best end-to-end combinations. Returns up to k
    // whole routes through the forced points, in cost order.
    paths = FindForcedPaths(graph, sources, goals, forced, k, options);
  } else if (k == 1) {
    const ShortestPath best = FindShortestPathMulti(graph, sources, goals, options);
    if (best.found && !best.vertices.empty()) {
      paths.push_back(best);
    }
  } else {
    paths = FindKShortestPathsMulti(graph, sources, goals, k, options);
  }
  if (paths.empty()) {
    return Result<Routes>::Err(Error(ErrorCode::kNoRoute, "no route between endpoints"));
  }

  // Look up a connection fix's seed cost among an endpoint's seeded fixes.
  auto seed_of = [](const std::vector<SeededEndpoint>& eps, int vertex) {
    for (const SeededEndpoint& e : eps) {
      if (e.vertex == vertex) {
        return e.cost;
      }
    }
    return 0.0;
  };

  // Classify how an endpoint attached to the network. A plan's connections are
  // homogeneous (all from a procedure build, or all DCT fallback), so this is a
  // per-endpoint verdict: a procedure was used; else procedures exist but none
  // reached the network (radar vectors); else no procedure data at all.
  auto connection_kind = [](const EndpointPlan& plan) {
    if (plan.used_procedures) {
      return ConnectionKind::kProcedure;
    }
    if (plan.has_procedures) {
      return ConnectionKind::kRadarVectors;
    }
    return ConnectionKind::kDirect;
  };
  const ConnectionKind dep_kind = connection_kind(dep);
  const ConnectionKind arr_kind = connection_kind(arr);

  Routes routes;
  routes.reserve(paths.size());
  for (const ShortestPath& p : paths) {
    if (p.vertices.empty()) {
      continue;
    }
    const int dep_fix = p.vertices.front();
    const int arr_fix = p.vertices.back();
    const double dep_seed = seed_of(sources, dep_fix);
    const double arr_seed = seed_of(goals, arr_fix);

    // Procedure selection depends on the candidate's own fix pair, which may
    // differ across candidates, so resolve it per path.
    std::string sid_name;
    std::string dep_rwy;
    std::vector<std::string> sid_options;
    SelectProcedures(dep, dep_fix, sid_name, dep_rwy, sid_options);
    std::string star_name;
    std::string arr_rwy;
    std::vector<std::string> star_options;
    SelectProcedures(arr, arr_fix, star_name, arr_rwy, star_options);

    Route route = MakeRoute(*builder_, graph, p, dep.airport_icao, arr.airport_icao, sid_name,
                            star_name, dep_seed, arr_seed);
    route.sid = sid_name;
    route.dep_runway = dep_rwy;
    route.sid_options = sid_options;
    route.star = star_name;
    route.arr_runway = arr_rwy;
    route.star_options = star_options;
    route.dep_connection = dep_kind;
    route.arr_connection = arr_kind;
    route.forced_points = forced_echo;
    routes.push_back(std::move(route));
  }
  return Result<Routes>::Ok(std::move(routes));
}

namespace {

// A resolved waypoint token during route parsing: its vertex and coordinate.
struct ResolvedFix {
  int vertex = -1;
  Coordinate coord;
};

}  // namespace

Result<Route> NavDatabase::ParseRoute(const std::string& route_str) const {
  if (!builder_) {
    return Result<Route>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }
  const std::vector<std::string> tokens = TokenizeRoute(route_str);
  if (tokens.empty()) {
    return Result<Route>::Err(Error(ErrorCode::kNoRoute, "empty route string"));
  }
  const NavGraph& graph = builder_->graph();

  // Resolve a bare waypoint ident (or IDENT/REGION) to the match nearest a
  // reference coordinate; connectivity along the route disambiguates naturally
  // because we always resolve against the previous point. Returns vertex -1 if
  // no non-airport match exists.
  auto resolve_fix = [&](const std::string& token, const Coordinate& ref) -> ResolvedFix {
    const size_t slash = token.find('/');
    if (slash != std::string::npos) {
      const int v = builder_->VertexByIdent(Ident(token.substr(0, slash), token.substr(slash + 1)));
      if (v < 0 || builder_->IsAirport(v)) {
        return {};
      }
      return {v, graph.CoordOf(v)};
    }
    int best = -1;
    double best_d = 0.0;
    for (const int v : builder_->VerticesByIdent(token)) {
      if (builder_->IsAirport(v)) {
        continue;
      }
      const double d = ref.DistanceTo(graph.CoordOf(v));
      if (best < 0 || d < best_d) {
        best = v;
        best_d = d;
      }
    }
    if (best < 0) {
      return {};
    }
    return {best, graph.CoordOf(best)};
  };

  // Walk airway `name` from `from` to `to`, following only edges whose
  // designators include `name` (Dijkstra restricted to that airway). Returns the
  // intermediate + destination vertices (excluding `from`) in order, or empty if
  // the airway does not connect them. Small, bounded search per airway.
  auto expand_airway = [&](const std::string& name, int from, int to) -> std::vector<int> {
    const int n = graph.VertexCount();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    using QN = std::pair<double, int>;
    std::priority_queue<QN, std::vector<QN>, std::greater<>> pq;
    dist[from] = 0.0;
    pq.push({0.0, from});
    while (!pq.empty()) {
      const auto [d, u] = pq.top();
      pq.pop();
      if (d > dist[u]) {
        continue;
      }
      if (u == to) {
        break;
      }
      for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
        if (e->airway_id == 0) {
          continue;  // DCT edge is not on any named airway
        }
        bool on_airway = false;
        for (const std::string& d2 : SplitDesignators(builder_->AirwayName(e->airway_id))) {
          if (d2 == name) {
            on_airway = true;
            break;
          }
        }
        if (!on_airway) {
          continue;
        }
        const double nd = d + e->distance_nm;
        if (nd < dist[e->to]) {
          dist[e->to] = nd;
          prev[e->to] = u;
          pq.push({nd, e->to});
        }
      }
    }
    if (std::isinf(dist[to])) {
      return {};  // airway does not connect from -> to
    }
    std::vector<int> chain;
    for (int at = to; at != from && at != -1; at = prev[at]) {
      chain.push_back(at);
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
  };

  Route route;
  std::vector<int> point_vertices;  // graph vertices for the enroute points

  // --- Optional leading departure airport. ---
  size_t i = 0;
  std::string dep_airport;
  if (builder_->VertexByAirport(tokens.front()) >= 0) {
    dep_airport = tokens.front();
    i = 1;
  }

  // --- Optional trailing arrival airport. ---
  std::string arr_airport;
  size_t end = tokens.size();
  if (end > i + 1 && builder_->VertexByAirport(tokens.back()) >= 0) {
    arr_airport = tokens.back();
    end = tokens.size() - 1;
  }

  // Reference coordinate for disambiguation: the departure airport if present,
  // else world origin (the first fix then resolves to its globally nearest
  // match, refined by connectivity on subsequent fixes).
  Coordinate ref = dep_airport.empty()
                       ? Coordinate{}
                       : graph.CoordOf(builder_->VertexByAirport(dep_airport));

  // --- Optional leading SID and trailing STAR (named procedures). ---
  // Recognized only adjacent to their airport and only if that airport actually
  // publishes the named procedure; otherwise the token is treated as a fix.
  auto airport_has_procedure = [&](const std::string& icao, const std::string& proc_name,
                                   ProcedureType type) -> bool {
    if (icao.empty()) {
      return false;
    }
    const CifpData* cifp = ProceduresFor(icao);
    if (cifp == nullptr) {
      return false;
    }
    for (const Procedure& p : cifp->procedures) {
      if (p.type == type && p.name == proc_name) {
        return true;
      }
    }
    return false;
  };

  std::string sid_name;
  if (i < end && airport_has_procedure(dep_airport, tokens[i], ProcedureType::kSid)) {
    sid_name = tokens[i];
    ++i;
  }
  std::string star_name;
  if (end > i && airport_has_procedure(arr_airport, tokens[end - 1], ProcedureType::kStar)) {
    star_name = tokens[end - 1];
    --end;
  }

  // --- Middle: FIX (AWY FIX | DCT FIX)* --------------------------------------
  // Track the previous fix vertex/coord; connectors (airway names, "DCT") apply
  // to the hop from the previous fix to the next.
  int prev_vertex = -1;
  bool expect_fix = true;
  std::string pending_connector;  // "" until a connector is seen; "DCT" or airway

  auto add_point = [&](int vertex) {
    point_vertices.push_back(vertex);
    route.points.push_back(RoutePoint{builder_->IdentOf(vertex).ident, graph.CoordOf(vertex)});
    prev_vertex = vertex;
    ref = graph.CoordOf(vertex);
  };

  for (; i < end; ++i) {
    const std::string& tok = tokens[i];
    const bool is_airway = airway_index_.find(tok) != airway_index_.end();

    if (expect_fix) {
      // Expecting a fix. A leading connector before any fix is an error.
      const ResolvedFix rf = resolve_fix(tok, ref);
      if (rf.vertex < 0) {
        return Result<Route>::Err(Error(
            ErrorCode::kNoRoute, "token '" + tok + "' is not a known waypoint at this position"));
      }
      if (prev_vertex < 0) {
        // First fix: just record it.
        add_point(rf.vertex);
      } else if (pending_connector == "DCT" || pending_connector.empty()) {
        // Direct leg from the previous fix.
        const double d = graph.CoordOf(prev_vertex).DistanceTo(rf.coord);
        route.legs.push_back(
            RouteLeg{builder_->IdentOf(prev_vertex).ident, builder_->IdentOf(rf.vertex).ident,
                     "DCT", d, {}});
        route.total_distance_nm += d;
        add_point(rf.vertex);
      } else {
        // Airway leg: expand the airway from prev to this fix.
        const std::vector<int> chain = expand_airway(pending_connector, prev_vertex, rf.vertex);
        if (chain.empty()) {
          return Result<Route>::Err(
              Error(ErrorCode::kNoRoute, "airway '" + pending_connector + "' does not connect " +
                                             builder_->IdentOf(prev_vertex).ident + " to " + tok));
        }
        int hop_from = prev_vertex;
        for (const int v : chain) {
          const double d = graph.CoordOf(hop_from).DistanceTo(graph.CoordOf(v));
          route.legs.push_back(RouteLeg{builder_->IdentOf(hop_from).ident,
                                        builder_->IdentOf(v).ident, pending_connector, d, {}});
          route.total_distance_nm += d;
          add_point(v);
          hop_from = v;
        }
      }
      pending_connector.clear();
      expect_fix = false;
    } else {
      // Expecting a connector: an airway name or DCT.
      if (tok == "DCT") {
        pending_connector = "DCT";
      } else if (is_airway) {
        pending_connector = tok;
      } else {
        // Two fixes in a row with no connector: treat as an implicit DCT so
        // "FIX FIX" is accepted (common in filed plans), then re-handle this
        // token as a fix.
        pending_connector = "DCT";
        --i;  // reprocess tok as a fix on the next iteration
      }
      expect_fix = true;
    }
  }

  if (point_vertices.empty()) {
    return Result<Route>::Err(Error(ErrorCode::kNoRoute, "route has no waypoints"));
  }
  if (!expect_fix && !pending_connector.empty()) {
    // A trailing connector with no following fix (e.g. "... PSB J60").
    return Result<Route>::Err(
        Error(ErrorCode::kNoRoute, "route ends with '" + pending_connector + "' but no fix"));
  }

  // --- Prepend the departure airport / SID and append the arrival / STAR. ---
  if (!dep_airport.empty()) {
    const int apt = builder_->VertexByAirport(dep_airport);
    const double d = graph.CoordOf(apt).DistanceTo(graph.CoordOf(point_vertices.front()));
    route.points.insert(route.points.begin(),
                        RoutePoint{dep_airport, graph.CoordOf(apt)});
    route.legs.insert(route.legs.begin(),
                      RouteLeg{dep_airport, builder_->IdentOf(point_vertices.front()).ident,
                               sid_name.empty() ? "DCT" : sid_name, d, {}});
    route.total_distance_nm += d;
    route.sid = sid_name;
  }
  if (!arr_airport.empty()) {
    const int apt = builder_->VertexByAirport(arr_airport);
    const double d = graph.CoordOf(point_vertices.back()).DistanceTo(graph.CoordOf(apt));
    route.points.push_back(RoutePoint{arr_airport, graph.CoordOf(apt)});
    route.legs.push_back(RouteLeg{builder_->IdentOf(point_vertices.back()).ident, arr_airport,
                                  star_name.empty() ? "DCT" : star_name, d, {}});
    route.total_distance_nm += d;
    route.star = star_name;
  }

  // Rebuild the canonical filed route string from the resolved legs (folds
  // consecutive same-airway legs, matching FindRoutes output).
  const std::string first_point = route.points.empty() ? "" : route.points.front().ident;
  route.route_string = BuildRouteString(first_point, route.legs);
  return Result<Route>::Ok(std::move(route));
}

std::vector<MsaSector> NavDatabase::MsaForAirport(const std::string& icao) const {
  const std::string up = ToUpper(icao);
  std::vector<MsaSector> out;
  for (const MsaSector& s : msa_) {
    if (s.airport_icao == up) {
      out.push_back(s);
    }
  }
  return out;
}

void NavDatabase::BuildAirwayIndex() {
  if (!builder_) {
    return;
  }
  const NavGraph& graph = builder_->graph();
  const int vcount = graph.VertexCount();
  for (int u = 0; u < vcount; ++u) {
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->airway_id == 0) {
        continue;  // synthetic DCT edge, not a named airway
      }
      const std::string& name = builder_->AirwayName(e->airway_id);
      const AirwayLeg leg{builder_->IdentOf(u).ident, builder_->IdentOf(e->to).ident,
                          e->distance_nm, EdgeIsHigh(*e), e->base_fl, e->top_fl};
      // A stored name may be a concurrency ("A593-Y592"): register the segment
      // under each designator so a lookup by any of them finds it. A single
      // airway splits to itself, so this is a no-op for the common case.
      for (const std::string& designator : SplitDesignators(name)) {
        AirwayInfo& info = airway_index_[designator];
        if (info.name.empty()) {
          info.name = designator;
        }
        info.segments.push_back(leg);
      }
    }
  }
}

std::vector<std::vector<WaypointInfo>> NavDatabase::LookupWaypoints(
    const std::vector<std::string>& idents) const {
  std::vector<std::vector<WaypointInfo>> out(idents.size());
  if (!builder_) {
    return out;
  }
  for (size_t i = 0; i < idents.size(); ++i) {
    const std::vector<int> vertices = builder_->VerticesByIdent(ToUpper(idents[i]));
    for (const int v : vertices) {
      // Airports share the ident namespace but are looked up via LookupAirports;
      // skip them here so a bare ICAO does not masquerade as a waypoint match.
      if (builder_->IsAirport(v)) {
        continue;
      }
      const Ident& id = builder_->IdentOf(v);
      out[i].push_back(WaypointInfo{id.ident, id.region, builder_->graph().CoordOf(v),
                                    builder_->KindOf(v), builder_->OnNetwork(v)});
    }
  }
  return out;
}

std::vector<std::optional<AirportInfo>> NavDatabase::LookupAirports(
    const std::vector<std::string>& icaos) const {
  std::vector<std::optional<AirportInfo>> out(icaos.size());
  if (!builder_) {
    return out;
  }
  for (size_t i = 0; i < icaos.size(); ++i) {
    const std::string up = ToUpper(icaos[i]);
    const int v = builder_->VertexByAirport(up);
    if (v < 0) {
      continue;
    }
    const Ident& id = builder_->IdentOf(v);
    out[i] = AirportInfo{id.ident, id.region, builder_->graph().CoordOf(v),
                         builder_->ElevationOf(v), ProceduresFor(up) != nullptr};
  }
  return out;
}

std::vector<std::optional<AirportProcedures>> NavDatabase::LookupProcedures(
    const std::vector<std::string>& icaos) const {
  std::vector<std::optional<AirportProcedures>> out(icaos.size());
  for (size_t i = 0; i < icaos.size(); ++i) {
    const std::string up = ToUpper(icaos[i]);
    const CifpData* cifp = ProceduresFor(up);
    if (cifp == nullptr) {
      continue;
    }
    AirportProcedures ap;
    ap.icao = up;
    ap.procedures.reserve(cifp->procedures.size());
    for (const Procedure& p : cifp->procedures) {
      ap.procedures.push_back(ProcedureSummary{p.type, p.name, p.transition_ident, p.runway});
    }
    out[i] = std::move(ap);
  }
  return out;
}

std::vector<std::optional<AirwayInfo>> NavDatabase::LookupAirways(
    const std::vector<std::string>& names) const {
  std::vector<std::optional<AirwayInfo>> out(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    auto it = airway_index_.find(ToUpper(names[i]));
    if (it != airway_index_.end()) {
      out[i] = it->second;
    }
  }
  return out;
}

}  // namespace bf
