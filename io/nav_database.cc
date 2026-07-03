#include "io/nav_database.h"

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/graph/nav_graph.h"
#include "core/routing/route_string.h"
#include "core/version.h"
#include "io/cache/bfdb_cache.h"
#include "io/cache/cifp_cache.h"
#include "io/cache/nav_detail_cache.h"
#include "io/graph_builder.h"
#include "io/loaders/xplane/cifp/cifp_parser.h"
#include "io/loaders/xplane/xplane_loader.h"

namespace bf {

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
  // Build the detail archive from the same parse (navaid_details/hold_fixes are
  // still in `data`; mora/msa were moved out above but those two were not).
  db.detail_archive_ = NavDetailArchive::FromData(data.value());
  db.BuildAirwayIndex();
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<NavDatabase> NavDatabase::OpenCached(const std::string& bfdb_path,
                                            const std::string& data_dir,
                                            const std::string& cifp_db_path, CifpLoad cifp_load) {
  Result<GraphArchive> archive = BfdbCache::Read(bfdb_path);
  if (!archive) {
    return Result<NavDatabase>::Err(std::move(archive).error());
  }
  GraphArchive& arc = archive.value();
  NavDatabase db;
  // The CIFP directory: an explicit override wins, else the build-time dir.
  db.data_dir_ = data_dir.empty() ? arc.data_dir : data_dir;
  db.cycle_ = arc.cycle;
  db.build_ = arc.build;
  db.mora_ = std::move(arc.mora);
  db.msa_ = std::move(arc.msa);
  db.builder_ = std::make_unique<GraphBuilder>(GraphBuilder::FromArchive(std::move(arc)));

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

  // Look for a sibling "<stem>_detail.bfdb" next to the graph cache. Absence is fine.
  {
    std::filesystem::path p(bfdb_path);
    const std::string detail_path =
        (p.parent_path() / (p.stem().string() + "_detail.bfdb")).string();
    if (std::filesystem::exists(detail_path)) {
      Result<NavDetailArchive> detail = NavDetailCache::Open(detail_path);
      if (!detail) {
        return Result<NavDatabase>::Err(std::move(detail).error());
      }
      db.detail_archive_ = std::move(detail).value();
    }
  }

  db.BuildAirwayIndex();
  return Result<NavDatabase>::Ok(std::move(db));
}

Result<void> NavDatabase::WriteCache(const std::string& out_path) const {
  if (!builder_) {
    return Result<void>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }
  GraphArchive archive = builder_->ToArchive(data_dir_, cycle_, build_);
  archive.program_semver = kBravoFinderVersion;
  archive.source_loader = "xplane";  // the only loader today; recorded as provenance
  archive.mora = mora_;
  archive.msa = msa_;
  return BfdbCache::Write(out_path, archive);
}

Result<uint32_t> NavDatabase::WriteCifpCache(const std::string& out_path,
                                             const std::string& source_loader) const {
  // Parse the full CIFP set on demand (heavy, ~100 MB), then hand the parsed
  // per-airport data to the source-agnostic cache writer. Keeping the parse here
  // (not in CifpCache) means the cache layer never depends on X-Plane's on-disk
  // layout, so a future loader can feed CifpCache::Build the same way.
  Result<std::vector<AirportProcedureData>> procedures = XPlaneLoader::LoadProcedures(data_dir_);
  if (!procedures) {
    return Result<uint32_t>::Err(std::move(procedures).error());
  }
  return CifpCache::Build(procedures.value(), out_path, source_loader, cycle_, build_,
                          kBravoFinderVersion);
}

Result<void> NavDatabase::WriteDetailCache(const std::string& out_path,
                                           const std::string& source_loader) const {
  if (!detail_archive_.has_value()) {
    return Result<void>::Err(Error(ErrorCode::kDataMissing, "no navaid detail archive to write"));
  }
  return NavDetailCache::Build(out_path, *detail_archive_, source_loader, kBravoFinderVersion);
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
      const AirwayLeg leg{builder_->IdentOf(u).ident,
                          builder_->IdentOf(e->to).ident,
                          e->distance_nm,
                          EdgeIsHigh(*e),
                          e->base_fl,
                          e->top_fl};
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

}  // namespace bf
