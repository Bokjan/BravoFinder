#pragma once

#include <string>

#include "core/result.h"

namespace bf {

struct GraphSnapshot;  // io/cache/graph_snapshot.h
class ByteWriter;      // io/cache/byte_io.h
class StringPool;      // io/cache/byte_io.h

// Encode/decode the graph SECTION of a unified `.bfdb`, back and forth between a
// GraphSnapshot and its on-disk byte body. This codec owns only the "struct <->
// bytes" mapping for the graph payload: it does no file I/O, writes no header,
// and carries no format version of its own. The unified container (unified_cache
// .h) owns the file, the header, the section table, the shared string pool, and
// the single container-level format version.
//
// String references (idents, airway names, MSA fix names) are interned into the
// caller-provided StringPool on Encode and resolved against the caller-provided
// pool blob on Decode, so the graph section shares one global pool with the CIFP
// and detail sections.
//
// The body is explicit, fixed-width, little-endian, and uses IEEE-754 bit
// patterns for floats, so bytes produced on one platform (x86-64, ARM) decode
// identically on another.
class GraphCodec {
 public:
  // Append the graph section body to `w`, interning strings into `pool`. Returns
  // an error if the snapshot exceeds format limits (e.g. > 65535 airway names)
  // or its arrays are internally inconsistent.
  static Result<void> Encode(const GraphSnapshot& snapshot, ByteWriter& w, StringPool& pool);

  // Decode a graph section body (`data`/`size`) into a GraphSnapshot, resolving
  // string references against the global pool blob (`pool`/`pool_len`). Returns
  // kCacheCorrupt if the body is malformed/truncated or a reference is out of
  // range.
  static Result<GraphSnapshot> Decode(const char* data, size_t size, const char* pool,
                                      size_t pool_len);
};

}  // namespace bf
