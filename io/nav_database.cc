#include "io/nav_database.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "core/constraints/altitude_constraints.h"
#include "core/constraints/mora_constraint.h"
#include "core/graph/yen_kshortest.h"
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
};

// Format a procedure reference as "NAME.TRANSITION" (or just "NAME" when the
// transition is empty / the common segment).
std::string FormatRef(const ProcedureRef& ref) {
  if (ref.transition.empty()) {
    return ref.name;
  }
  return ref.name + "." + ref.transition;
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
    route.legs.push_back(RouteLeg{arr_fix_id, arr_label, star.empty() ? "DCT" : star, arr_seed, {}});
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
        plan.used_procedures = !plan.connections.empty();
      }
      if (plan.connections.empty()) {
        // No usable procedures: fall back to DCT links to the nearest
        // on-network waypoints. The airport stays the route endpoint; the
        // connecting leg shows "DCT" since no procedure was selected.
        plan.connections = ProcedureConnector::BuildDctFallback(apt, *builder_, 5);
      }
      return plan;
    }
    const int wp = builder_->VertexByIdent(up);
    if (wp >= 0) {
      plan.connections.push_back(Connection{wp, 0.0, {}});
    }
    return plan;
  };

  EndpointPlan dep = plan_endpoint(request.departure, /*departure=*/true);
  if (dep.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown departure: " + request.departure));
  }
  EndpointPlan arr = plan_endpoint(request.arrival, /*departure=*/false);
  if (arr.connections.empty()) {
    return Result<Routes>::Err(
        Error(ErrorCode::kAirportNotFound, "unknown arrival: " + request.arrival));
  }

  // Assemble the active constraints from the request.
  AltitudeBandConstraint altitude_band;
  MoraConstraint mora(mora_);
  LevelPreferenceConstraint level_pref;
  SearchOptions options;
  options.request = &request;
  if (request.cruise_fl.has_value()) {
    options.constraints.push_back(&altitude_band);
    options.constraints.push_back(&mora);
  }
  if (request.level != LevelPreference::kNone) {
    options.constraints.push_back(&level_pref);
  }
  // Airports must not be transit nodes: their synthetic DCT links would let the
  // search cut through an unrelated airport (e.g. ...MIE DCT KMIE SNKPT...).
  // Endpoints connect via seeded connection fixes, not airport vertices, so
  // blocking all airport vertices as intermediate nodes is safe.
  const GraphBuilder* builder_ptr = builder_.get();
  options.node_blocked = [builder_ptr](int v) { return builder_ptr->IsAirport(v); };

  const NavGraph& graph = builder_->graph();
  const std::vector<SeededEndpoint> sources = ProcedureConnector::ToEndpoints(dep.connections);
  const std::vector<SeededEndpoint> goals = ProcedureConnector::ToEndpoints(arr.connections);

  // Find up to k candidate routes. Unlike the earlier scheme that fixed a single
  // best connection-fix pair and only varied the enroute portion between them,
  // the multi-endpoint Yen lets each candidate join through a different SID/STAR
  // connection fix, so the alternatives can use genuinely different procedures.
  // Both forms report distance_nm with both seed costs already included.
  const int k = std::max(1, request.k);
  std::vector<ShortestPath> paths;
  if (k == 1) {
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
    routes.push_back(std::move(route));
  }
  return Result<Routes>::Ok(std::move(routes));
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

}  // namespace bf
