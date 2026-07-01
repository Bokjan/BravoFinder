#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/cache/cifp_cache.h"
#include "io/loaders/xplane/cifp/cifp_parser.h"

namespace bf {

class GraphBuilder;

// How a CIFP procedure cache is loaded by OpenCached.
enum class CifpLoad {
  kOnDemand,  // read only the header+directory; fetch each airport's segment on
              // first use (~1.5 MB resident; best for one-shot CLI queries)
  kEager,     // deserialize every airport's procedures at Open (~100 MB resident,
              // then lock-free; best for servers / batch routing)
};

// The top-level navigation database: owns the loaded data and the route graph,
// and answers route queries. Self-contained with no global/static state, so
// multiple instances (e.g. different AIRAC cycles) can coexist safely.
//
// Thread-safety contract: after Open() succeeds, an instance is read-only except
// for an internally synchronized procedure cache. FindRoutes() and
// MsaForAirport() are const and may be called concurrently from multiple threads
// on the SAME instance. The only shared mutable state is procedure_cache_, which
// is guarded by cache_mutex_; everything else (graph, MORA, MSA) is immutable
// after Open().
class NavDatabase {
 public:
  NavDatabase();
  ~NavDatabase();
  NavDatabase(NavDatabase&&) noexcept;
  NavDatabase& operator=(NavDatabase&&) noexcept;

  // Load X-Plane data from `data_dir` and build the route graph. Returns the
  // ready database or an Error.
  static Result<NavDatabase> Open(const std::string& data_dir);

  // Load a prebuilt graph from a `.bfdb` cache file, skipping all parsing and
  // graph construction (seconds -> milliseconds). `data_dir` locates the CIFP
  // files that ProceduresFor still reads on demand; if empty, the data_dir
  // recorded at build time is used. `cifp_db_path` points to a `nav_cifp.bfdb`
  // procedure cache; if empty, a sibling `<stem>_cifp.bfdb` next to `bfdb_path`
  // is used when present, else procedures fall back to CIFP files under
  // data_dir. `cifp_load` selects on-demand (default) or eager loading of the
  // procedure cache (see CifpLoad). Returns an Error if the graph cache is
  // missing, corrupt, or of an incompatible format.
  static Result<NavDatabase> OpenCached(const std::string& bfdb_path,
                                        const std::string& data_dir = "",
                                        const std::string& cifp_db_path = "",
                                        CifpLoad cifp_load = CifpLoad::kOnDemand);

  // Serialize the built graph and metadata to a `.bfdb` cache file. Called by
  // `bf build` after Open(). Returns an Error if the file cannot be written.
  Result<void> WriteCache(const std::string& out_path) const;

  // Serialize every airport's CIFP procedures to a segmented `nav_cifp.bfdb`
  // procedure cache, so deployment needs only the cache files (not the CIFP
  // directory). `source_loader` is recorded as provenance. Returns the number
  // of airports written, or an Error. Reads from data_dir_/CIFP.
  Result<uint32_t> WriteCifpCache(const std::string& out_path,
                                  const std::string& source_loader) const;

  // Find up to request.k candidate routes, ordered best-first, honoring the
  // request's altitude/level constraints. Endpoints resolve as airport ICAO
  // first, then waypoint ident; case-insensitive. Returns an Error if an
  // endpoint is unknown or no route exists.
  Result<std::vector<Route>> FindRoutes(const RouteRequest& request) const;

  // The minimum sector altitudes published for `icao` (terminal-area MSA), or
  // an empty span if none. Case-insensitive on the ICAO code.
  std::vector<MsaSector> MsaForAirport(const std::string& icao) const;

 private:
  // Load (and cache) an airport's CIFP procedures on demand. Returns nullptr if
  // the airport has no CIFP file. The cache accumulates across queries so a
  // session of related queries pays each airport's parse cost only once.
  //
  // Thread-safe: cache_mutex_ guards only the map lookup/insert, never the disk
  // parse, so concurrent queries for different airports parse in parallel. The
  // returned pointer stays valid for the database's lifetime: the cache is
  // append-only (no erase) and stores unique_ptr values, so a CifpData's heap
  // address is stable even when a concurrent insert rehashes the map.
  const CifpData* ProceduresFor(const std::string& icao) const;

  // AIRAC provenance, carried into the .bfdb cache header.
  uint32_t cycle_ = 0;
  uint32_t build_ = 0;
  std::unique_ptr<GraphBuilder> builder_;
  MoraGrid mora_;
  std::vector<MsaSector> msa_;
  std::string data_dir_;
  // Optional CIFP procedure cache. When present, ProceduresFor fetches segments
  // from it instead of parsing CIFP/<ICAO>.dat files. Immutable after Open, so
  // it needs no lock (its Fetch opens an independent ifstream per call).
  std::optional<CifpArchive> cifp_archive_;
  // When true, procedure_cache_ was fully populated at Open and is frozen: reads
  // hit existing entries only, so ProceduresFor skips the lock entirely (no
  // insert => no rehash => no data race). When false (on-demand), the cache is
  // filled lazily under cache_mutex_.
  bool cifp_eager_ = false;
  // Procedure cache. In on-demand mode it is lazily filled by FindRoutes
  // (logically const) under cache_mutex_; in eager mode it is filled once at
  // Open, then read lock-free. The mutex is held in a unique_ptr so NavDatabase
  // stays movable (std::mutex is not movable; the defaulted move operations need
  // a movable member).
  mutable std::unordered_map<std::string, std::unique_ptr<CifpData>> procedure_cache_;
  mutable std::unique_ptr<std::mutex> cache_mutex_;
};

}  // namespace bf
