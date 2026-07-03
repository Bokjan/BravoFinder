#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/result.h"
#include "io/loaders/xplane12/cifp/cifp_parser.h"

namespace bf {

// A read-only, on-demand archive of per-airport CIFP procedure data, backed by
// a `nav_cifp.bfdb` file. Opening reads only the header and directory (an
// ICAO -> {offset, length} index) into memory; the per-airport segments stay on
// disk and are deserialized one at a time by Fetch().
//
// Thread-safety: after Open, the archive is immutable (the directory is fixed).
// Fetch() opens its own ifstream per call and shares no mutable state, so it is
// safe to call concurrently from multiple threads. This preserves NavDatabase's
// contract B without any locking here.
class CifpArchive {
 public:
  CifpArchive() = default;

  // Deserialize the segment for `icao` (case-sensitive; callers upper-case).
  // Returns std::nullopt if the airport is not in the archive or its segment is
  // corrupt. Each call performs independent file I/O.
  std::optional<CifpData> Fetch(const std::string& icao) const;

  // Deserialize every airport's segment in one pass, returning an ICAO -> data
  // map. Used for eager loading; reads the whole file once. Segments that fail
  // to deserialize are skipped.
  std::unordered_map<std::string, CifpData> FetchAll() const;

  // Whether the archive contains a segment for `icao`.
  bool Has(const std::string& icao) const { return index_.count(icao) != 0; }

  uint32_t cycle() const { return cycle_; }
  uint32_t build() const { return build_; }
  const std::string& source_loader() const { return source_loader_; }
  const std::string& program_semver() const { return program_semver_; }

 private:
  friend class CifpCache;

  std::string path_;  // the nav_cifp.bfdb this archive reads segments from
  std::unordered_map<std::string, std::pair<uint64_t, uint32_t>> index_;  // icao -> (offset, len)
  uint32_t cycle_ = 0;
  uint32_t build_ = 0;
  std::string source_loader_;
  std::string program_semver_;
};

// Builds and opens the `nav_cifp.bfdb` procedure cache: a portable, segmented
// binary of every airport's parsed CIFP data, indexed by ICAO for on-demand
// loading. Same little-endian, cross-platform format family as graph_cache.
class CifpCache {
 public:
  // v1: initial segmented format (magic "BFCP").
  static constexpr uint32_t kFormatVersion = 1;

  // Write already-parsed per-airport procedure data as segments into `out_path`,
  // recording the given provenance in the header. Source-agnostic: the caller
  // (a loader) supplies (ICAO, CifpData) pairs, so this never touches any data
  // source's on-disk layout. Returns the number of airports written, or an Error.
  static Result<uint32_t> Build(const std::vector<std::pair<std::string, CifpData>>& procedures,
                                const std::string& out_path, const std::string& source_loader,
                                uint32_t cycle, uint32_t build, const std::string& program_semver);

  // Open an archive: read the header and directory into memory. Segments are
  // fetched lazily. Returns kDataMissing if the file is absent or malformed.
  static Result<CifpArchive> Open(const std::string& path);
};

}  // namespace bf
