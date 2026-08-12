// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/navdb/nav_database.h"

#include <algorithm>
#include <filesystem>
#include <format>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/base/log.h"
#include "core/base/string_util.h"
#include "core/domain/fixed_string.h"
#include "core/graph/nav_graph.h"
#include "core/graph/yen_kshortest.h"
#include "core/routing/route_string.h"
#include "core/version.h"
#include "io/build/graph_builder.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/graph_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"
#include "io/cache/unified_cache.h"
#include "io/loaders/loader.h"
#include "io/loaders/loader_registry.h"
#include "io/navdb/constraint_assembly.h"
#include "io/navdb/endpoint_planner.h"
#include "io/navdb/forced_router.h"
#include "io/navdb/route_assembler.h"

namespace bf {

NavDatabase::NavDatabase() : cache_mutex_(std::make_unique<std::mutex>()) {}
NavDatabase::~NavDatabase() = default;
NavDatabase::NavDatabase(NavDatabase&&) noexcept = default;
NavDatabase& NavDatabase::operator=(NavDatabase&&) noexcept = default;

Result<NavDatabase> NavDatabase::Open(const std::string& source_dir,
                                      const std::string& loader_name) {
  Result<std::unique_ptr<Loader>> loader = MakeLoader(loader_name);
  if (!loader) {
    return Result<NavDatabase>::Err(std::move(loader).error());
  }
  Result<NavData> data = loader.value()->LoadNavData(source_dir);
  if (!data) {
    return Result<NavDatabase>::Err(std::move(data).error());
  }
  NavDatabase db;
  db.loader_ = std::move(loader).value();
  db.source_dir_ = source_dir;
  db.source_loader_ = db.loader_->name();
  db.capabilities_ = CapabilitiesForLoader(db.source_loader_);
  db.cycle_ = data.value().cycle;
  db.mora_ = std::move(data.value().mora);
  db.msa_ = std::move(data.value().msa);
  Result<GraphBuilder> builder = GraphBuilder::Build(data.value());
  if (!builder) {
    return Result<NavDatabase>::Err(std::move(builder).error());
  }
  db.builder_ = std::make_unique<GraphBuilder>(std::move(builder).value());
  // Reject a build that overflowed the uint16 airway_id space (see
  // GraphBuilder::airway_overflow); real AIRAC data never triggers this.
  if (db.builder_->airway_overflow()) {
    return Result<NavDatabase>::Err(
        Error(ErrorCode::kSerializationError,
              std::format("too many distinct airway names (> {}) for the uint16 airway_id space",
                          std::numeric_limits<uint16_t>::max())));
  }
  // Build the detail archive from the same parse (navaid_details/hold_fixes are
  // still in `data`; mora/msa were moved out above but those two were not).
  db.detail_archive_ = std::make_unique<NavDetailArchive>(NavDetailArchive::FromData(data.value()));
  db.BuildAirwayIndex();
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<NavDatabase> NavDatabase::OpenCached(const std::string& bfdb_path, CifpLoad cifp_load) {
  Result<UnifiedData> unified = UnifiedCache::Open(bfdb_path);
  if (!unified) {
    return Result<NavDatabase>::Err(std::move(unified).error());
  }
  UnifiedData& u = unified.value();

  NavDatabase db;
  db.cycle_ = u.header.cycle;
  db.source_loader_ = u.header.source_loader;
  db.capabilities_ = CapabilitiesForLoader(db.source_loader_);
  db.mora_ = std::move(u.graph.mora);
  db.msa_ = std::move(u.graph.msa);
  db.builder_ = std::make_unique<GraphBuilder>(GraphBuilder::FromSnapshot(std::move(u.graph)));

  // CIFP procedures come from the file's CIFP section, if present. Its absence
  // is fine -- an airport then simply reports no procedures (the cached path
  // never reads source .dat files).
  if (u.cifp.has_value()) {
    if (cifp_load == CifpLoad::kEager) {
      // Deserialize every airport up front into the procedure cache, then freeze
      // it: subsequent ProceduresFor calls only read existing entries, so they
      // need no lock (thread-safety contract holds with no shared mutable state).
      // A single corrupt segment fails the whole Open -- never drop airports and
      // pretend the cache is complete (that would silently DCT-fallback).
      Result<std::unordered_map<std::string, CifpData>> all = u.cifp->FetchAll();
      if (!all) {
        return Result<NavDatabase>::Err(std::move(all).error());
      }
      db.procedure_cache_.reserve(all.value().size());
      for (auto& entry : all.value()) {
        db.procedure_cache_.emplace(entry.first,
                                    std::make_unique<CifpData>(std::move(entry.second)));
      }
      db.cifp_eager_ = true;
      // The archive is not retained: everything is already in the cache.
    } else {
      db.cifp_archive_ = std::make_unique<CifpArchive>(std::move(*u.cifp));
    }
  }

  // Navaid detail comes from the file's detail section, if present. Absence is fine.
  if (u.detail.has_value()) {
    db.detail_archive_ = std::make_unique<NavDetailArchive>(std::move(*u.detail));
  }

  db.BuildAirwayIndex();
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<uint32_t> NavDatabase::WriteUnified(const std::string& out_path) const {
  if (!builder_) {
    return Result<uint32_t>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }
  // The CIFP section is mandatory: a cache without procedures cannot resolve
  // SID/STAR, so it is always written. Requires a loader (a database opened from
  // a cache has none).
  if (loader_ == nullptr) {
    return Result<uint32_t>::Err(
        Error(ErrorCode::kDataMissing, "no loader (database opened from a cache)"));
  }

  // Graph section: the built graph plus MORA/MSA (owned by NavDatabase).
  GraphSnapshot snapshot = builder_->ToSnapshot();
  snapshot.mora = mora_;
  snapshot.msa = msa_;

  // CIFP section: parse the full procedure set via the loader (~100 MB), then
  // hand the parsed per-airport data to the source-agnostic codec. Keeping the
  // parse in the loader means the cache layer never depends on any source's
  // on-disk layout.
  Result<std::vector<AirportProcedureData>> procedures = loader_->LoadProcedures(source_dir_);
  if (!procedures) {
    return Result<uint32_t>::Err(std::move(procedures).error());
  }
  std::vector<AirportProcedureData> cifp_procedures = std::move(procedures).value();

  UnifiedCache::BuildInput input;
  input.graph = &snapshot;
  input.cifp = &cifp_procedures;
  input.detail = detail_archive_.get();
  input.header.cycle = cycle_;
  input.header.program_version = kBravoFinderVersion;
  input.header.source_loader = loader_->name();
  input.header.data_dir = source_dir_;

  Result<void> written = UnifiedCache::Build(out_path, input);
  if (!written) {
    return Result<uint32_t>::Err(std::move(written).error());
  }
  return Result<uint32_t>::Ok(static_cast<uint32_t>(cifp_procedures.size()));
}

Result<const CifpData*> NavDatabase::ProceduresFor(const std::string& icao) const {
  // Moved-from instances have a null cache_mutex_ (the move ops are = default).
  // They are never meant to be queried (see the class comment), but unlike
  // LookupWaypoints/LookupAirports this path has no !builder_ early return, so
  // guard the lock explicitly: return "no procedures" rather than null-deref.
  if (!cache_mutex_) {
    return Result<const CifpData*>::Ok(nullptr);
  }
  // Eager mode: the cache was fully populated at Open and is now frozen, so a
  // plain read needs no lock (no concurrent insert can rehash it). A miss means
  // the airport simply has no procedures.
  if (cifp_eager_) {
    auto it = procedure_cache_.find(icao);
    return Result<const CifpData*>::Ok(it != procedure_cache_.end() ? it->second.get() : nullptr);
  }
  // Fast path: return a cached result (including a cached "no procedures"
  // nullptr) under a brief lock.
  {
    std::lock_guard<std::mutex> guard(*cache_mutex_);
    auto it = procedure_cache_.find(icao);
    if (it != procedure_cache_.end()) {
      return Result<const CifpData*>::Ok(it->second.get());
    }
  }
  // Parse outside the lock so concurrent queries for different airports do not
  // serialize on disk I/O. Two threads racing on the same airport will both
  // parse (harmless, redundant work). Source: the CIFP cache archive if one is
  // loaded (positional read on a shared handle, thread-safety contract safe),
  // else the loader parsing a source .dat on demand (Open path). With neither,
  // the airport has no procedures. Corrupt archive segments return Err and are
  // NOT inserted as nullptr (that would permanently mask cache damage as
  // "no SID/STAR").
  std::unique_ptr<CifpData> stored;
  if (cifp_archive_) {
    Result<std::optional<CifpData>> fetched = cifp_archive_->Fetch(icao);
    if (!fetched) {
      BF_LOG_ERROR("CIFP fetch failed for {}: {}", icao, fetched.error().message);
      return Result<const CifpData*>::Err(std::move(fetched).error());
    }
    if (fetched.value().has_value()) {
      stored = std::make_unique<CifpData>(std::move(fetched.value()).value());
    }
  } else if (loader_) {
    std::optional<CifpData> parsed = loader_->LoadProcedure(source_dir_, icao);
    if (parsed.has_value()) {
      stored = std::make_unique<CifpData>(std::move(parsed).value());
    }
  }
  // Re-lock and insert. try_emplace keeps the first inserted value if another
  // thread won the race, so a previously returned pointer is never invalidated;
  // the losing thread's parsed copy is simply discarded. Return the value that
  // actually lives in the cache.
  std::lock_guard<std::mutex> guard(*cache_mutex_);
  auto it = procedure_cache_.try_emplace(icao, std::move(stored)).first;
  return Result<const CifpData*>::Ok(it->second.get());
}

void NavDatabase::BuildAirwayIndex() {
  if (!builder_) {
    return;
  }
  const NavGraph& graph = builder_->graph();
  const int vcount = graph.VertexCount();
  // Track which directed legs each designator has already registered, so an
  // exact-duplicate segment in the source data is not stored twice. A kBoth
  // airway yields distinct u->v and v->u legs (different from/to), which are
  // kept: AirwayInfo represents reversals honestly. The key is the full leg
  // identity (endpoints + level + FL band), not just (from, to).
  auto leg_key = [](const AirwayLeg& l) {
    // Use the char '\0'/'\1' overload of operator+, NOT the C-string literals
    // "\0"/"\1": "\0" is an EMPTY string (length 0), which would drop the
    // separator entirely for low airways and let (to="WPT", base_fl=120) collide
    // with (to="WPT1", base_fl=20) on the same key "...WPT120..." -- silently
    // discarding the second leg as a false duplicate. A char appends the real
    // null/0x01 byte and keeps every field boundary intact.
    return std::string(l.from.View()) + '\0' + std::string(l.to.View()) + (l.high ? '\1' : '\0') +
           std::to_string(l.base_fl) + '\0' + std::to_string(l.top_fl);
  };
  // Accumulate into a hash map, then freeze into the sorted resident vector.
  // Build is one-shot at Open; the sorted form is what LookupAirways / ParseRoute
  // share for the rest of the database lifetime.
  std::unordered_map<std::string, AirwayInfo> built;
  std::unordered_map<std::string, std::unordered_set<std::string>> seen;
  for (int u = 0; u < vcount; ++u) {
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->airway_id == 0) {
        continue;  // synthetic DCT edge, not a named airway
      }
      const std::string& name = builder_->AirwayName(e->airway_id);
      Result<FixedName8> from = FixedName8::From(builder_->IdentOf(u).ident);
      Result<FixedName8> to = FixedName8::From(builder_->IdentOf(e->to).ident);
      if (!from || !to) {
        BF_LOG_WARN("airway index: skipping edge with invalid endpoint name: {}",
                    !from ? from.error().message : to.error().message);
        continue;
      }
      const AirwayLeg leg{std::move(from).value(),
                          std::move(to).value(),
                          e->distance_nm,
                          e->level == AirwayLevel::kHigh,
                          e->base_fl,
                          e->top_fl};
      // A stored name may be a concurrency ("A593-Y592"): register the segment
      // under each designator so a lookup by any of them finds it. A single
      // airway splits to itself, so this is a no-op for the common case.
      for (const std::string& designator : SplitDesignators(name)) {
        if (!seen[designator].insert(leg_key(leg)).second) {
          continue;  // exact-duplicate directed leg already registered
        }
        AirwayInfo& info = built[designator];
        if (info.name.empty()) {
          info.name = designator;
        }
        info.segments.push_back(leg);
      }
    }
  }
  airway_index_.clear();
  airway_index_.reserve(built.size());
  for (auto& entry : built) {
    const std::string& designator = entry.first;
    // Designators are <=5 chars in real AIRAC data; FixedName8 caps at 7. An
    Result<FixedName8> key = FixedName8::From(designator);
    if (!key) {
      BF_LOG_WARN("airway index: skipping designator '{}': {}", designator, key.error().message);
      continue;
    }
    airway_index_.emplace_back(std::move(key).value(), std::move(entry.second));
  }
  std::sort(airway_index_.begin(), airway_index_.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
}

const AirwayInfo* NavDatabase::FindAirway(std::string_view name) const {
  Result<FixedName8> key_result = FixedName8::From(name);
  if (!key_result) {
    return nullptr;
  }
  const FixedName8& key = key_result.value();
  auto it = std::lower_bound(
      airway_index_.begin(), airway_index_.end(), key,
      [](const std::pair<FixedName8, AirwayInfo>& e, const FixedName8& k) { return e.first < k; });
  if (it != airway_index_.end() && it->first == key) {
    return &it->second;
  }
  return nullptr;
}

Result<std::vector<Route>> NavDatabase::FindRoutes(const RouteRequest& request) const {
  using Routes = std::vector<Route>;
  if (!builder_) {
    return Result<Routes>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }

  // Keep the callable alive: CifpLookup is a non-owning function_ref.
  const auto cifp_fn = [this](const std::string& icao) { return ProceduresFor(icao); };
  const CifpLookup cifp_lookup{cifp_fn};

  Result<EndpointPlan> dep_r =
      EndpointPlanner::Plan(*builder_, request, request.departure, /*departure=*/true, cifp_lookup);
  if (!dep_r) {
    return Result<Routes>::Err(std::move(dep_r).error());
  }
  EndpointPlan dep = std::move(dep_r).value();
  if (dep.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kProcedureNotFound,
                                     "departure airport " + request.departure +
                                         " has no SID matching '" + request.departure_sid + "'"));
  }
  if (dep.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown departure airport: " + request.departure + " (expected an ICAO code, e.g. KLAX)"));
  }
  Result<EndpointPlan> arr_r =
      EndpointPlanner::Plan(*builder_, request, request.arrival, /*departure=*/false, cifp_lookup);
  if (!arr_r) {
    return Result<Routes>::Err(std::move(arr_r).error());
  }
  EndpointPlan arr = std::move(arr_r).value();
  if (arr.named_procedure_unmatched) {
    return Result<Routes>::Err(Error(ErrorCode::kProcedureNotFound,
                                     "arrival airport " + request.arrival +
                                         " has no STAR matching '" + request.arrival_star + "'"));
  }
  if (arr.connections.empty()) {
    return Result<Routes>::Err(Error(
        ErrorCode::kAirportNotFound,
        "unknown arrival airport: " + request.arrival + " (expected an ICAO code, e.g. KLAX)"));
  }

  Result<ConstraintBundle> constraints_r = ConstraintAssembly::Build(request, *builder_, mora_);
  if (!constraints_r) {
    return Result<Routes>::Err(std::move(constraints_r).error());
  }
  ConstraintBundle constraints = std::move(constraints_r).value();
  SearchOptions& options = constraints.options;
  const std::vector<int>& avoid_vertices = constraints.avoid_vertices;

  const NavGraph& graph = builder_->graph();
  std::vector<SeededEndpoint> sources =
      EndpointPlanner::ToSearchEndpoints(dep.connections, /*soft_prefer_star=*/false);
  const bool arr_soft_prefer = EndpointPlanner::SoftPreferStarActive(arr);
  std::vector<SeededEndpoint> goals =
      EndpointPlanner::ToSearchEndpoints(arr.connections, arr_soft_prefer);
  // Drop any seeded connection fix the request asks to avoid: it would otherwise
  // slip through as a search start/end, which AvoidConstraint cannot catch.
  if (!avoid_vertices.empty()) {
    auto drop_avoided = [&](std::vector<SeededEndpoint>& eps) {
      eps.erase(std::remove_if(eps.begin(), eps.end(),
                               [&](const SeededEndpoint& e) {
                                 return std::binary_search(avoid_vertices.begin(),
                                                           avoid_vertices.end(), e.vertex);
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
          ForcedRouter::ResolvePoint(*builder_, token, dep_coord, arr_coord, echo, is_airport);
      if (v < 0) {
        const std::string why =
            is_airport ? "' is an airport, not an enroute waypoint" : "' is not a known waypoint";
        return Result<Routes>::Err(
            Error(ErrorCode::kRouteParseError, "forced point '" + token + why));
      }
      if (std::binary_search(avoid_vertices.begin(), avoid_vertices.end(), v)) {
        return Result<Routes>::Err(Error(ErrorCode::kRouteParseError,
                                         "forced point '" + token + "' is also in the avoid list"));
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
    paths = ForcedRouter::FindPaths(graph, sources, goals, forced, k, options);
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

  return Result<Routes>::Ok(RouteAssembler::Assemble(*builder_, graph, paths, dep, arr,
                                                     arr_soft_prefer, options, forced_echo));
}

}  // namespace bf
