// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "core/domain/msa.h"
#include "core/query/query_types.h"
#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/loaders/loader_capabilities.h"

namespace bf {

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
// Opaque to SDK consumers: implementation state and private helpers live behind
// `impl_` (see the internal nav_database_impl.h, which is not installed).
//
// Thread-safety contract: after Open() succeeds, an instance is read-only except
// for an internally synchronized procedure cache. EVERY const query method --
// FindRoutes(), MsaForAirport(), ParseRoute(), and the batch lookups
// (LookupWaypoints, LookupAirports, LookupAirways, LookupHolds, ...) -- is safe
// to call concurrently from multiple threads on the SAME instance. The only
// shared mutable state is the procedure cache inside Impl, guarded by its mutex;
// everything else (graph, MORA, MSA) is immutable after Open().
//
// Moved-from instances are NOT queryable: a moved-from database has a null
// `impl_` and any query that needs state returns empty / "not loaded". Current
// call sites never query a moved-from instance; treat "moved from" as
// "consumed", not "reset for reuse".
class NavDatabase {
 public:
  NavDatabase();
  ~NavDatabase();
  NavDatabase(NavDatabase&&) noexcept;
  NavDatabase& operator=(NavDatabase&&) noexcept;

  NavDatabase(const NavDatabase&) = delete;
  NavDatabase& operator=(const NavDatabase&) = delete;

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
  uint32_t cycle() const;

  // Loader that produced this database (`bf build --loader` / bfdb header).
  // Empty when unknown. Does not by itself change FindRoutes behaviour.
  const std::string& source_loader() const;

  // What the source can faithfully express (see LoaderCapabilities). Taken from
  // the live loader on Open, or restored from the .bfdb header on OpenCached.
  // Exposed for callers/UIs; FindRoutes does not currently degrade constraints
  // from these.
  const LoaderCapabilities& capabilities() const;

  // Find up to request.k candidate routes, ordered best-first, honoring the
  // request's altitude/level constraints. Endpoints are airport ICAO codes
  // (case-insensitive); bare waypoint idents are rejected. Returns an Error if
  // an endpoint is unknown or no route exists.
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
  // one-element batch. All are const and safe for concurrent use per thread-safety contract.
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
  // per entry when the airport is unknown or has no CIFP data. Returns
  // Err(kCacheCorrupt) if any requested airport's cached CIFP segment is
  // unreadable — never conflated with "no procedures".
  Result<std::vector<std::optional<AirportProcedures>>> LookupProcedures(
      const std::vector<std::string>& icaos) const;

  // Per-leg detail of one named procedure at an airport. `procedure_name` is a
  // published name (e.g. "DEEZZ5"); the result holds every transition of that
  // name with its ordered legs (course/distance/altitude plus RNP, turn
  // direction, and speed limit). Ok(nullopt) when the airport is unknown, has
  // no CIFP data, or publishes no procedure of that name. Err(kCacheCorrupt)
  // when the airport's CIFP segment is present but unreadable. Case-insensitive.
  Result<std::optional<AirportProcedureDetail>> LookupProcedureDetail(
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
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bf
