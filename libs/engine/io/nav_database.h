// SPDX-License-Identifier: LGPL-3.0-or-later
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
#include "core/domain/procedure.h"
#include "core/query/query_types.h"
#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/nav_detail_codec.h"

namespace bf {

class GraphBuilder;
class Loader;

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
// for an internally synchronized procedure cache. EVERY const query method --
// FindRoutes(), MsaForAirport(), ParseRoute(), the batch lookups (LookupWaypoints,
// LookupAirports, LookupAirways, LookupHolds, ...) and ProceduresFor() -- is safe
// to call concurrently from multiple threads on the SAME instance. The only shared
// mutable state is procedure_cache_, which is guarded by cache_mutex_; everything
// else (graph, MORA, MSA) is immutable after Open().
//
// Moved-from instances are NOT queryable: the move ops are `= default`, so a
// moved-from database has a null cache_mutex_ (a unique_ptr member) and any query
// touching the procedure cache would null-deref. Current call sites never query a
// moved-from instance; treat "moved from" as "consumed", not "reset for reuse".
class NavDatabase {
 public:
  NavDatabase();
  ~NavDatabase();
  NavDatabase(NavDatabase&&) noexcept;
  NavDatabase& operator=(NavDatabase&&) noexcept;

  // Load navigation source data from `source_dir` and build the route graph.
  // `loader_name` selects the source loader (see MakeLoader / the loader
  // registry for the available names) and is recorded as `source_loader`
  // provenance in any caches written.
  // Returns the ready database or an Error (including an unknown loader name).
  static Result<NavDatabase> Open(const std::string& source_dir,
                                  const std::string& loader_name = "xplane12");

  // Load a prebuilt database from a unified `.bfdb` file, skipping all parsing
  // and graph construction (seconds -> milliseconds). Graph, CIFP procedures and
  // navaid detail all come from the one file's sections; if the CIFP section is
  // absent, an airport simply reports no procedures (the cached path never reads
  // source `.dat` files). `cifp_load` selects on-demand (default) or eager
  // loading of the procedure section (see CifpLoad). Returns an Error if the file
  // is missing, corrupt, or of an incompatible format.
  static Result<NavDatabase> OpenCached(const std::string& bfdb_path,
                                        CifpLoad cifp_load = CifpLoad::kOnDemand);

  // Serialize the whole database -- graph, CIFP procedures, and navaid detail --
  // into ONE unified `.bfdb` file. Called by `bf build` after Open(). The CIFP
  // procedure section is always written: building a cache without procedures is
  // not useful (routes cannot resolve SID/STAR), so the section is mandatory.
  // Reads the CIFP procedures via the loader from the source dir passed to Open,
  // so a database opened from a cache (no loader) cannot write -- returns an
  // Error in that case. Returns the number of airports written into the CIFP
  // section, or an Error if the file cannot be written.
  Result<uint32_t> WriteUnified(const std::string& out_path) const;

  // AIRAC provenance parsed from the source data (or restored from a cache): the
  // cycle number (e.g. 2601). Zero when the source carried no parsable
  // provenance. (The X-Plane-only `build` stamp is no longer tracked.)
  uint32_t cycle() const { return cycle_; }

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

  // Per-leg detail of one named procedure at an airport. `procedure_name` is a
  // published name (e.g. "DEEZZ5"); the result holds every transition of that
  // name with its ordered legs (course/distance/altitude plus RNP, turn
  // direction, and speed limit). nullopt when the airport is unknown, has no
  // CIFP data, or publishes no procedure of that name. Case-insensitive.
  std::optional<AirportProcedureDetail> LookupProcedureDetail(
      const std::string& icao, const std::string& procedure_name) const;

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
  // the airport has no procedures. The cache accumulates across queries so a
  // session of related queries pays each airport's parse cost only once. The
  // source is the loaded CIFP archive if one is present (OpenCached path), else
  // the loader parsing a source `.dat` on demand (Open path); if neither is
  // available the airport simply has no procedures.
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

  // AIRAC provenance, carried into the .bfdb container header.
  uint32_t cycle_ = 0;
  std::unique_ptr<GraphBuilder> builder_;
  MoraGrid mora_;
  std::vector<MsaSector> msa_;
  // The source loader, and the source directory it parses. Both set only on the
  // Open() path (where raw data is parsed); null/empty on the OpenCached() path,
  // which reads only prebuilt caches and never needs a loader.
  std::unique_ptr<Loader> loader_;
  std::string source_dir_;
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
