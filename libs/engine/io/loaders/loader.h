// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/domain/procedure.h"
#include "core/result.h"
#include "io/nav_data.h"

namespace bf {

// One airport's parsed CIFP data, keyed by ICAO. LoadProcedures returns these in
// loader-defined order; the ICAO identifies the airport.
using AirportProcedureData = std::pair<std::string, CifpData>;

// Abstract navigation-data source. A Loader turns some on-disk representation
// (X-Plane 12 native ".dat" files, or a SQLite navigation database -- the
// available loaders live under io/loaders/ and register themselves with the
// loader registry) into BravoFinder's domain types. A loader is only needed where raw source data
// must be parsed -- the `bf build` and NavDatabase::Open paths. The cached path (OpenCached) reads
// prebuilt .bfdb caches and needs no loader.
//
// Loaders are stateless and their methods are const, so a single instance can
// serve concurrent parses.
class Loader {
 public:
  virtual ~Loader() = default;

  // Parse the enroute dataset (waypoints, navaids, airways, airport reference
  // positions, MORA/MSA, holds, navaid detail) from `source_dir` into NavData,
  // or an Error. This is the lightweight path: it does NOT parse CIFP terminal
  // procedures (see LoadProcedures / LoadProcedure).
  virtual Result<NavData> LoadNavData(const std::string& source_dir) const = 0;

  // Parse every airport's CIFP terminal procedures from `source_dir` into
  // per-airport data. This is the heavy path (~100 MB for a full cycle) used
  // only by `bf build` to feed CifpCache::Build. Returns an Error if the source
  // has no procedure data at all; individual unreadable airports are skipped.
  virtual Result<std::vector<AirportProcedureData>> LoadProcedures(
      const std::string& source_dir) const = 0;

  // Parse a single airport's CIFP procedures from `source_dir` on demand, or
  // nullopt if that airport has no procedure data. Used by NavDatabase::Open's
  // lazy procedure cache so a route/query only parses the airports it touches.
  virtual std::optional<CifpData> LoadProcedure(const std::string& source_dir,
                                                const std::string& icao) const = 0;

  // The loader's stable name, recorded as `source_loader` provenance in caches
  // and selected by `bf build --loader`. Must round-trip through MakeLoader.
  virtual std::string name() const = 0;
};

}  // namespace bf
