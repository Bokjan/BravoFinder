// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/result.h"
#include "io/cache/pread_file.h"
#include "io/loaders/xplane12/cifp_parser.h"

namespace bf {

class ByteWriter;  // io/cache/byte_io.h
class StringPool;  // io/cache/byte_io.h

// A read-only, on-demand view of per-airport CIFP procedure data, backed by the
// CIFP section of a unified `.bfdb` file. It holds the directory (an ICAO ->
// {absolute file offset, length} index) and a copy of the container's global
// string pool in memory; the per-airport segments stay on disk and are
// deserialized one at a time by Fetch().
//
// Thread-safety: after construction the archive is immutable. Fetch() reads its
// segment via a positional read (pread / ReadFile with an explicit offset) on a
// shared read-only handle that keeps no mutable cursor, so it is safe to call
// concurrently from multiple threads. The owned pool blob is const after
// construction. This preserves NavDatabase's contract B without any locking
// here. The owned handle makes the archive move-only.
class CifpArchive {
 public:
  CifpArchive() = default;

  // Deserialize the segment for `icao` (case-sensitive; callers upper-case).
  // Returns std::nullopt if the airport is not in the archive or its segment is
  // corrupt. Reads only that segment, at its offset, from the shared handle;
  // resolves string references against the in-memory global pool.
  std::optional<CifpData> Fetch(const std::string& icao) const;

  // Deserialize every airport's segment, returning an ICAO -> data map. Used for
  // eager loading. Segments that fail to deserialize are skipped.
  std::unordered_map<std::string, CifpData> FetchAll() const;

  // Whether the archive contains a segment for `icao`.
  bool Has(const std::string& icao) const { return index_.count(icao) != 0; }

 private:
  friend class CifpCodec;

  PreadFile file_;  // shared read-only handle on the unified .bfdb
  std::unordered_map<std::string, std::pair<uint64_t, uint32_t>> index_;  // icao -> (abs off, len)
  std::vector<uint8_t> pool_;  // owned copy of the container's global string pool blob
};

// Encode/decode the CIFP SECTION of a unified `.bfdb`: a segmented, on-demand
// store of every airport's parsed procedure data, indexed by ICAO. This codec
// owns only the "struct <-> bytes" mapping for the CIFP payload plus the logic
// to open the section into a CifpArchive; it carries no format version of its
// own (the container owns the single version).
//
// Section body layout (all string refs point into the container's global pool):
//   airport_count : U32
//   directory     : airport_count * (icao_off U32, icao_len U32,
//                                     seg_offset U64, seg_len U32)
//                   seg_offset is RELATIVE to the CIFP section start.
//   segments      : each a bare body (no per-segment pool); refs into global pool
class CifpCodec {
 public:
  // Append the CIFP section body (directory + segments) to `w`, interning ICAO
  // codes and every segment's strings into the shared `pool`. Source-agnostic:
  // the caller supplies already-parsed (ICAO, CifpData) pairs. Returns the
  // number of airports encoded.
  static Result<uint32_t> Encode(const std::vector<std::pair<std::string, CifpData>>& procedures,
                                 ByteWriter& w, StringPool& pool);

  // Open the CIFP section of the unified file at `path` into a CifpArchive: read
  // the directory (bounded by the section), resolve ICAO codes against the
  // global pool blob, and hold a pread handle for lazy segment fetches. Takes an
  // owned copy of the global pool blob (needed by Fetch). `section_offset` /
  // `section_length` locate the CIFP section within the file.
  static Result<CifpArchive> OpenSection(const std::string& path, uint64_t section_offset,
                                         uint64_t section_length, std::vector<uint8_t> pool_blob);
};

}  // namespace bf
