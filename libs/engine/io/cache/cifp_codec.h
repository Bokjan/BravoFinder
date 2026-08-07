// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/domain/fixed_string.h"
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
// construction. This preserves NavDatabase's thread-safety contract without any locking
// here. The owned handle makes the archive move-only.
class CifpArchive {
 public:
  CifpArchive() = default;

  // Deserialize the segment for `icao` (case-sensitive; callers upper-case).
  // Ok(nullopt) = airport not in the archive (no procedures). Err(kCacheCorrupt)
  // = I/O failure, CRC mismatch, or segment deserialize failure -- never conflated
  // with "missing". Reads only that segment, at its offset, from the shared handle;
  // resolves string references against the in-memory global pool.
  Result<std::optional<CifpData>> Fetch(const std::string& icao) const;

  // Deserialize every airport's segment. Any single corrupt/unreadable segment
  // fails the whole call with kCacheCorrupt (eager Open must not silently drop
  // airports and pretend the cache is complete).
  Result<std::unordered_map<std::string, CifpData>> FetchAll() const;

  // Whether the archive contains a segment for `icao`.
  bool Has(const std::string& icao) const { return Find(icao) != nullptr; }

 private:
  friend class CifpCodec;

  // One airport's segment location + integrity check, looked up by ICAO.
  struct SegmentLoc {
    uint64_t abs_off;  // absolute file offset of the segment body
    uint32_t len;      // segment body length in bytes
    uint32_t crc;      // CRC-32C of the segment body (0 for an empty segment)
  };

  // Binary-search the sorted ICAO index. Over-long keys cannot match any stored
  // ICAO (cap 7, real ICAOs <=4) and return null without asserting.
  const SegmentLoc* Find(const std::string& icao) const;

  PreadFile file_;  // shared read-only handle on the unified .bfdb
  // Sorted by ICAO (FixedName8); frozen after OpenSection. Replaces an
  // unordered_map: ~14k entries, read-only after Open, lookup is cold-path.
  std::vector<std::pair<FixedName8, SegmentLoc>> index_;
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
//                                     seg_offset U64, seg_len U32, seg_crc U32)
//                   seg_offset is RELATIVE to the CIFP section start.
//                   seg_crc is the CRC-32C of that segment's body bytes.
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
  // the directory (bounded by the section), verify the directory prefix against
  // `section_crc` (the CRC stored in the section-table row, covering airport_count
  // + directory rows -- the lazy segments carry their own per-segment CRC and are
  // not covered here), resolve ICAO codes against the global pool blob, and hold
  // a pread handle for lazy segment fetches. Takes an owned copy of the global
  // pool blob (needed by Fetch). `section_offset` / `section_length` locate the
  // CIFP section within the file.
  static Result<CifpArchive> OpenSection(const std::string& path, uint64_t section_offset,
                                         uint64_t section_length, uint32_t section_crc,
                                         std::vector<uint8_t> pool_blob);

  // Bytes a section-table CRC covers for the CIFP section: the airport_count U32
  // plus the directory rows. The per-airport segments are NOT covered here -- they
  // are lazy and carry their own CRC in their directory row, so a section-level
  // CRC over the whole body would force reading every segment at Open, defeating
  // lazy fetch. Used by the container to compute the directory-prefix CRC at Build.
  static size_t DirectoryPrefixLen(uint32_t airport_count);
};

}  // namespace bf
