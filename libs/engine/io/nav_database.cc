// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/nav_database.h"

#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "core/graph/nav_graph.h"
#include "core/routing/route_string.h"
#include "core/version.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/graph_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"
#include "io/cache/unified_cache.h"
#include "io/graph_builder.h"
#include "io/loaders/loader.h"
#include "io/loaders/loader_registry.h"

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
  db.cycle_ = data.value().cycle;
  db.mora_ = std::move(data.value().mora);
  db.msa_ = std::move(data.value().msa);
  db.builder_ = std::make_unique<GraphBuilder>(data.value());
  // Reject a build that overflowed the uint16 airway_id space (see
  // GraphBuilder::airway_overflow); real AIRAC data never triggers this.
  if (db.builder_->airway_overflow()) {
    return Result<NavDatabase>::Err(
        Error(ErrorCode::kParseError,
              "too many distinct airway names (>65535) for the uint16 airway_id space"));
  }
  // Build the detail archive from the same parse (navaid_details/hold_fixes are
  // still in `data`; mora/msa were moved out above but those two were not).
  db.detail_archive_ = NavDetailArchive::FromData(data.value());
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
      // need no lock (contract B holds with no shared mutable state).
      std::unordered_map<std::string, CifpData> all = u.cifp->FetchAll();
      db.procedure_cache_.reserve(all.size());
      for (auto& entry : all) {
        db.procedure_cache_.emplace(entry.first,
                                    std::make_unique<CifpData>(std::move(entry.second)));
      }
      db.cifp_eager_ = true;
      // The archive is not retained: everything is already in the cache.
    } else {
      db.cifp_archive_ = std::move(u.cifp);
    }
  }

  // Navaid detail comes from the file's detail section, if present. Absence is fine.
  if (u.detail.has_value()) {
    db.detail_archive_ = std::move(u.detail);
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
  input.detail = detail_archive_.has_value() ? &detail_archive_.value() : nullptr;
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
  // loaded (an independent ifstream per fetch, contract-B safe), else the loader
  // parsing a source .dat on demand (Open path). With neither, the airport has
  // no procedures.
  std::unique_ptr<CifpData> stored;
  if (cifp_archive_.has_value()) {
    std::optional<CifpData> fetched = cifp_archive_->Fetch(icao);
    if (fetched.has_value()) {
      stored = std::make_unique<CifpData>(std::move(fetched).value());
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
  return it->second.get();
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
    return l.from + '\0' + l.to + (l.high ? '\1' : '\0') + std::to_string(l.base_fl) + '\0' +
           std::to_string(l.top_fl);
  };
  std::unordered_map<std::string, std::unordered_set<std::string>> seen;
  for (int u = 0; u < vcount; ++u) {
    for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
      if (e->airway_id == 0) {
        continue;  // synthetic DCT edge, not a named airway
      }
      const std::string& name = builder_->AirwayName(e->airway_id);
      const AirwayLeg leg{builder_->IdentOf(u).ident,
                          builder_->IdentOf(e->to).ident,
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
