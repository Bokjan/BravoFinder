// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Internal NavDatabase::Impl definition. NOT part of the SDK — do not install.
// Only nav_database.cc / parse.cc / query.cc may include this header.

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/base/attributes.h"
#include "core/domain/fixed_string.h"
#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/query/query_types.h"
#include "core/result.h"
#include "io/build/graph_builder.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/nav_detail_codec.h"
#include "io/loaders/loader.h"
#include "io/loaders/loader_capabilities.h"
#include "io/navdb/nav_database.h"

namespace bf {

struct NavDatabase::Impl {
  // AIRAC provenance, carried into the .bfdb container header.
  uint32_t cycle_ = 0;
  std::string source_loader_;
  LoaderCapabilities capabilities_{};
  std::unique_ptr<GraphBuilder> builder_;
  MoraGrid mora_;
  std::vector<MsaSector> msa_;
  // The source loader, and the source directory it parses. Both set only on the
  // Open() path (where raw data is parsed); null/empty on the OpenCached() path,
  // which reads only prebuilt caches and never needs a loader.
  std::unique_ptr<Loader> loader_;
  std::string source_dir_;
  // Airway designator -> its directed segments. Built once at Open, then
  // immutable, so reads are lock-free. Sorted by FixedName8 key for
  // binary search (designators are <=5 chars; ~10k entries, cold-path lookup).
  std::vector<std::pair<FixedName8, AirwayInfo>> airway_index_;
  // Optional CIFP procedure cache. When present, ProceduresFor fetches segments
  // from it instead of parsing CIFP/<ICAO>.dat files. Immutable after Open, so
  // it needs no lock (its Fetch opens an independent ifstream per call).
  std::unique_ptr<CifpArchive> cifp_archive_;
  // Optional navaid detail + hold cache, loaded eagerly at Open.
  // Immutable after Open; FindNavaids/FindHolds are const and lock-free.
  std::unique_ptr<NavDetailArchive> detail_archive_;
  // When true, procedure_cache_ was fully populated at Open and is frozen: reads
  // hit existing entries only, so ProceduresFor skips the lock entirely (no
  // insert => no rehash => no data race). When false (on-demand), the cache is
  // filled lazily under cache_mutex_.
  bool cifp_eager_ = false;
  // Procedure cache. In on-demand mode it is lazily filled by FindRoutes
  // (logically const) under cache_mutex_; in eager mode it is filled once at
  // Open, then read lock-free. The mutex is held in a unique_ptr so Impl (and
  // thus NavDatabase) stays movable (std::mutex is not movable).
  mutable std::unordered_map<std::string, std::unique_ptr<CifpData>> procedure_cache_;
  mutable std::unique_ptr<std::mutex> cache_mutex_ = std::make_unique<std::mutex>();

  // Load (and cache) an airport's CIFP procedures on demand. Ok(nullptr) if the
  // airport has no procedures. Err(kCacheCorrupt) if a cached segment is present
  // but unreadable/corrupt -- never cached as "no procedures".
  //
  // Thread-safe: cache_mutex_ guards only the map lookup/insert, never the disk
  // parse. The returned pointer stays valid for the owning NavDatabase's
  // lifetime (append-only unique_ptr cache).
  Result<const CifpData*> ProceduresFor(const std::string& icao) const BF_LIFETIMEBOUND;

  // Build the airway-name -> segments index by scanning every graph edge once.
  // Called at the end of Open/OpenCached; then frozen (read-only).
  void BuildAirwayIndex();

  // Binary-search airway_index_ by designator. Over-long names cannot match
  // (designators are <=5 chars) and return null without asserting.
  const AirwayInfo* FindAirway(std::string_view name) const;
};

}  // namespace bf
