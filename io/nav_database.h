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
#include "core/query/query_types.h"
#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/cache/cifp_cache.h"
#include "io/cache/nav_detail_cache.h"
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

  // AIRAC provenance parsed from the source data (or restored from a cache):
  // the cycle number (e.g. 2601) and build date stamp (e.g. 20260112). Zero
  // when the source carried no parsable provenance.
  uint32_t cycle() const { return cycle_; }
  uint32_t build() const { return build_; }

  // Serialize every airport's CIFP procedures to a segmented `nav_cifp.bfdb`
  // procedure cache, so deployment needs only the cache files (not the CIFP
  // directory). `source_loader` is recorded as provenance. Returns the number
  // of airports written, or an Error. Reads from data_dir_/CIFP.
  Result<uint32_t> WriteCifpCache(const std::string& out_path,
                                  const std::string& source_loader) const;

  // Serialize the navaid detail + hold archive to a `nav_..._detail.bfdb` side
  // cache. `source_loader` is recorded as provenance. Returns an Error if the
  // database has no detail archive (e.g. opened from a cache without one) or the
  // file cannot be written.
  Result<void> WriteDetailCache(const std::string& out_path,
                                const std::string& source_loader) const;

  // Find up to request.k candidate routes, ordered best-first, honoring the
  // request's altitude/level constraints. Endpoints resolve as airport ICAO
  // first, then waypoint ident; case-insensitive. Returns an Error if an
  // endpoint is unknown or no route exists.
  Result<std::vector<Route>> FindRoutes(const RouteRequest& request) const;

  // Parse and validate a filed-flight-plan route string -- the reverse of
  // FindRoutes. Given "[DEP] [SID] FIX (AWY FIX | DCT FIX)* [STAR] [ARR]", verify
  // each airway actually connects its bracketing fixes, expand the airways to
  // their intermediate points, and total the great-circle distance. Returns the
  // resolved Route (points / legs / route_string / total_distance_nm) or an
  // Error naming the offending token and why it failed. A named SID/STAR is
  // validated against the endpoint airport's procedures and shown as a single
  // connection leg, not expanded leg-by-leg.
  Result<Route> ParseRoute(const std::string& route_str) const;

  // The minimum sector altitudes published for `icao` (terminal-area MSA), or
  // an empty span if none. Case-insensitive on the ICAO code.
  std::vector<MsaSector> MsaForAirport(const std::string& icao) const;

  // --- Batch lookup API ------------------------------------------------------
  // Each lookup takes a list of keys and returns a result vector parallel to it,
  // where a miss is the empty element for that result shape: nullopt for the
  // optional-returning lookups, and an empty inner vector for LookupWaypoints
  // (which returns all region matches per ident). A single lookup is just a
  // one-element batch. All are const and safe for concurrent use per contract B.
  // Idents and ICAO codes are matched case-insensitively.

  // Waypoints / navaids by ident. Since an ident is reused across regions, each
  // input ident maps to a vector of every matching WaypointInfo (possibly empty
  // when the ident is unknown or names an airport). The outer vector is parallel
  // to `idents`; the inner vector holds all region matches for that ident.
  std::vector<std::vector<WaypointInfo>> LookupWaypoints(
      const std::vector<std::string>& idents) const;

  // Airports by ICAO code.
  std::vector<std::optional<AirportInfo>> LookupAirports(
      const std::vector<std::string>& icaos) const;

  // Published terminal procedures (SID/STAR/approach) by airport ICAO. nullopt
  // when the airport is unknown or has no CIFP data.
  std::vector<std::optional<AirportProcedures>> LookupProcedures(
      const std::vector<std::string>& icaos) const;

  // Airways by designator (e.g. "Y28"). Returns every directed segment carrying
  // that name. nullopt when no segment uses the name.
  std::vector<std::optional<AirwayInfo>> LookupAirways(const std::vector<std::string>& names) const;

  // Navaid detail attributes (freq/range/elev/heading) by ident. Returns all
  // region matches per ident (empty inner vector when not found or no detail
  // cache was loaded). Requires a detail cache opened via OpenCached.
  std::vector<std::vector<NavaidDetailInfo>> LookupNavaidDetails(
      const std::vector<std::string>& idents) const;

  // Holding patterns by fix ident. Returns all holds at that fix across all
  // regions and airports (empty inner vector when not found or no detail cache).
  std::vector<std::vector<HoldInfo>> LookupHolds(const std::vector<std::string>& fix_idents) const;

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

  // Build the airway-name -> segments index by scanning every graph edge once.
  // Called at the end of Open/OpenCached; the index is then frozen (read-only),
  // so LookupAirways needs no lock (contract B: immutable after Open).
  void BuildAirwayIndex();

  // AIRAC provenance, carried into the .bfdb cache header.
  uint32_t cycle_ = 0;
  uint32_t build_ = 0;
  std::unique_ptr<GraphBuilder> builder_;
  MoraGrid mora_;
  std::vector<MsaSector> msa_;
  std::string data_dir_;
  // Airway designator -> its directed segments. Built once at Open, then
  // immutable, so reads are lock-free.
  std::unordered_map<std::string, AirwayInfo> airway_index_;
  // Optional CIFP procedure cache. When present, ProceduresFor fetches segments
  // from it instead of parsing CIFP/<ICAO>.dat files. Immutable after Open, so
  // it needs no lock (its Fetch opens an independent ifstream per call).
  std::optional<CifpArchive> cifp_archive_;
  // Optional navaid detail + hold cache, loaded eagerly at Open.
  // Immutable after Open; FindNavaids/FindHolds are const and lock-free.
  std::optional<NavDetailArchive> detail_archive_;
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
