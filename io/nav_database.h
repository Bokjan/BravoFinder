#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/domain/mora_grid.h"
#include "core/domain/msa.h"
#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/loaders/xplane/cifp/cifp_parser.h"

namespace bf {

class GraphBuilder;

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

  std::unique_ptr<GraphBuilder> builder_;
  MoraGrid mora_;
  std::vector<MsaSector> msa_;
  std::string data_dir_;
  // Procedure cache, lazily filled by FindRoutes (logically const). Guarded by
  // cache_mutex_. The mutex is held in a unique_ptr so NavDatabase stays movable
  // (std::mutex is not movable; the defaulted move operations need a movable
  // member).
  mutable std::unordered_map<std::string, std::unique_ptr<CifpData>> procedure_cache_;
  mutable std::unique_ptr<std::mutex> cache_mutex_;
};

}  // namespace bf
