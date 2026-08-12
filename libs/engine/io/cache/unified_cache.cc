// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/unified_cache.h"

#include <cstdint>
#include <cstring>
#include <format>
#include <fstream>
#include <span>
#include <utility>
#include <vector>

#include "io/cache/byte_io.h"
#include "io/cache/cache_messages.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/crc32c.h"
#include "io/cache/graph_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"

namespace bf {

namespace {

constexpr char kMagic[4] = {'B', 'F', 'D', 'B'};

// Section types, stored in each section-table row.
constexpr uint32_t kSectionGraph = 1;
constexpr uint32_t kSectionCifp = 2;
constexpr uint32_t kSectionDetail = 3;

// Fixed section count: graph + cifp + detail. Absent sections are written with
// offset == length == 0 rather than dropped, so the table is a fixed size.
constexpr uint32_t kSectionCount = 3;
constexpr size_t kSectionRowSize = 4 + 8 + 8 + 4;  // type U32 + crc U32 + offset U64 + length U64

// A generous bound on the header prefix (magic + fixed U32s + three inline
// provenance strings). Real provenance strings are a few dozen bytes and a data
// dir is a filesystem path, so 8 KiB covers the whole prefix in one read; a
// string length pointing past what was read means the file is corrupt.
constexpr size_t kMaxHeaderPrefix = 8192;

// ofstream/ifstream speak char*; the cache layer's byte buffers are
// std::vector<uint8_t>. These two helpers centralize the reinterpret_cast so
// every read/write site stays byte-typed.
void WriteAll(std::ostream& f, std::span<const uint8_t> b) {
  f.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
}

bool ReadInto(std::istream& f, std::span<uint8_t> b) {
  if (!b.empty()) {
    f.read(reinterpret_cast<char*>(b.data()), static_cast<std::streamsize>(b.size()));
  }
  return static_cast<bool>(f);
}

}  // namespace

Result<void> UnifiedCache::Build(const std::string& path, const BuildInput& input) {
  if (input.graph == nullptr) {
    return Result<void>::Err(Error(ErrorCode::kInvalidArgument, "unified cache requires a graph"));
  }

  // One string pool shared by every section, so a string recurring across
  // sections (idents, ICAO codes, fix names) is stored once. All three codecs
  // intern into this pool; its blob is finalized only after every section body
  // is encoded.
  StringPool pool;

  std::vector<uint8_t> graph_body;
  {
    ByteWriter gw(graph_body);
    Result<void> enc = GraphCodec::Encode(*input.graph, gw, pool);
    if (!enc) {
      return enc;
    }
  }

  std::vector<uint8_t> cifp_body;
  const bool has_cifp = input.cifp != nullptr;
  uint32_t cifp_airport_count = 0;
  if (has_cifp) {
    ByteWriter cw(cifp_body);
    Result<uint32_t> enc = CifpCodec::Encode(*input.cifp, cw, pool);
    if (!enc) {
      return Result<void>::Err(std::move(enc).error());
    }
    cifp_airport_count = enc.value();
  }

  std::vector<uint8_t> detail_body;
  const bool has_detail = input.detail != nullptr;
  if (has_detail) {
    ByteWriter dw(detail_body);
    Result<void> enc = NavDetailCodec::Encode(*input.detail, dw, pool);
    if (!enc) {
      return enc;
    }
  }

  const std::span<const uint8_t> pool_blob = pool.blob();

  // The string pool's offset/length references and the pool_len header field are
  // uint32. If the deduplicated pool exceeds 4 GiB those values would silently
  // wrap, producing a cache file whose refs point at the wrong strings while
  // Open still reads it -- a silent corruption. Real AIRAC data is far under
  // this, so reject rather than truncate. (graph/cifp/detail section sizes are
  // uint64, so only the shared pool is bounded this way.)
  if (pool_blob.size() > 0xFFFFFFFFull) {
    return Result<void>::Err(
        Error(ErrorCode::kSerializationError,
              "unified cache string pool exceeds 4 GiB; the uint32 pool offset space "
              "cannot address it. This is unexpected for real AIRAC data."));
  }

  // Header buffer: magic, version, section_count, cycle, provenance strings,
  // capabilities (4x U8), pool_len, pool_crc. Built first so its size is known
  // before section offsets.
  std::vector<uint8_t> header;
  ByteWriter hw(header);
  hw.Bytes(reinterpret_cast<const uint8_t*>(kMagic), 4);
  hw.U32(kFormatVersion);
  hw.U32(kSectionCount);
  hw.U32(input.header.cycle);
  hw.Str(input.header.program_version);
  hw.Str(input.header.source_loader);
  hw.Str(input.header.data_dir);
  hw.U8(input.header.capabilities.airway_direction ? 1 : 0);
  hw.U8(input.header.capabilities.altitude_bands ? 1 : 0);
  hw.U8(input.header.capabilities.mora_grid ? 1 : 0);
  hw.U8(input.header.capabilities.msa_sectors ? 1 : 0);
  hw.U32(static_cast<uint32_t>(pool_blob.size()));
  // pool_crc guards the shared string pool -- every section's string refs
  // resolve into it, so a pool bit-flip silently corrupts text across all
  // sections while each section's own CRC (covering only its on-disk off/len
  // refs, not the pool text) stays valid. The pool is read once at Open, so a
  // single U32 here closes the largest hole in section-level CRC coverage.
  hw.U32(Crc32C::Compute(pool_blob));
  // A provenance string over 4 GiB cannot be length-prefixed by a U32; Str()
  // would have set the writer failed without writing. Fail through Result rather
  // than computing section offsets from a desynchronized header buffer.
  if (!hw.ok()) {
    return Result<void>::Err(Error(ErrorCode::kSerializationError,
                                   "cache header string exceeds the 4 GiB U32 length prefix; "
                                   "cannot serialize. This is unexpected for real AIRAC data."));
  }

  // Sections begin after [header][section table][pool]. Compute each present
  // section's absolute file offset in table order (graph, cifp, detail).
  const uint64_t sections_start =
      header.size() + static_cast<uint64_t>(kSectionCount) * kSectionRowSize + pool_blob.size();
  uint64_t cursor = sections_start;
  const uint64_t graph_off = cursor;
  cursor += graph_body.size();
  const uint64_t cifp_off = has_cifp ? cursor : 0;
  cursor += cifp_body.size();
  const uint64_t detail_off = has_detail ? cursor : 0;

  // Section CRCs. graph/detail cover their full body (eagerly read at Open);
  // CIFP covers only its directory prefix (count + directory rows) -- segments
  // are lazy and carry their own CRC in their directory row, so a section-level
  // CRC over the whole body would force reading every segment at Open.
  const uint32_t graph_crc = Crc32C::Compute(graph_body);
  const uint32_t cifp_crc = has_cifp ? Crc32C::Compute(std::span<const uint8_t>(cifp_body).subspan(
                                           0, CifpCodec::DirectoryPrefixLen(cifp_airport_count)))
                                     : 0;
  const uint32_t detail_crc = has_detail ? Crc32C::Compute(detail_body) : 0;

  std::vector<uint8_t> table;
  ByteWriter tw(table);
  tw.U32(kSectionGraph);
  tw.U32(graph_crc);
  tw.U64(graph_off);
  tw.U64(graph_body.size());
  tw.U32(kSectionCifp);
  tw.U32(cifp_crc);
  tw.U64(cifp_off);
  tw.U64(has_cifp ? cifp_body.size() : 0);
  tw.U32(kSectionDetail);
  tw.U32(detail_crc);
  tw.U64(detail_off);
  tw.U64(has_detail ? detail_body.size() : 0);

  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f.is_open()) {
    return Result<void>::Err(
        Error(ErrorCode::kDataMissing, std::format("cannot open .bfdb for writing: {}", path)));
  }
  WriteAll(f, header);
  WriteAll(f, table);
  WriteAll(f, pool_blob);
  WriteAll(f, graph_body);
  if (has_cifp) {
    WriteAll(f, cifp_body);
  }
  if (has_detail) {
    WriteAll(f, detail_body);
  }
  if (!f) {
    return Result<void>::Err(
        Error(ErrorCode::kSerializationError, std::format("failed writing .bfdb: {}", path)));
  }
  return Result<void>::Ok();
}

Result<UnifiedHeader> UnifiedCache::ReadHeader(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kDataMissing, std::format("cannot open .bfdb: {}", path)));
  }
  std::vector<uint8_t> buf(kMaxHeaderPrefix);
  ReadInto(f, buf);  // may short-read a tiny file; gcount() tells how much
  buf.resize(static_cast<size_t>(f.gcount()));
  if (buf.size() < 4) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kCacheCorrupt, std::format("truncated .bfdb: {}", path)));
  }
  if (std::memcmp(buf.data(), kMagic, 4) != 0) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kCacheCorrupt, WithRebuildHint("not a .bfdb file (bad magic)")));
  }
  ByteReader r(std::span<const uint8_t>(buf).subspan(4));
  if (r.U32() != kFormatVersion) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kFormatMismatch, WithRebuildHint("incompatible .bfdb format version")));
  }
  r.U32();  // section_count -- not needed to read the header fields
  UnifiedHeader header;
  header.cycle = r.U32();
  header.program_version = r.Str();
  header.source_loader = r.Str();
  header.data_dir = r.Str();
  header.capabilities.airway_direction = r.U8() != 0;
  header.capabilities.altitude_bands = r.U8() != 0;
  header.capabilities.mora_grid = r.U8() != 0;
  header.capabilities.msa_sectors = r.U8() != 0;
  if (!r.ok()) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kCacheCorrupt, WithRebuildHint("corrupt .bfdb header")));
  }
  return Result<UnifiedHeader>::Ok(std::move(header));
}

Result<UnifiedData> UnifiedCache::Open(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    return Result<UnifiedData>::Err(
        Error(ErrorCode::kDataMissing, std::format("cannot open .bfdb: {}", path)));
  }
  const std::streamoff file_size = f.tellg();
  f.seekg(0);

  auto bad = [&](const char* why) {
    return Result<UnifiedData>::Err(Error(ErrorCode::kCacheCorrupt, WithRebuildHint(why)));
  };

  if (file_size < 4) {
    return bad("truncated .bfdb");
  }

  // Read the fixed header + provenance strings + section table + global pool.
  // These are near the file start; the (large) section bodies are read
  // individually afterward, so this initial read stays small.
  auto readN = [&](std::vector<uint8_t>& dst, size_t n) {
    dst.resize(n);
    if (n > 0) {
      f.read(reinterpret_cast<char*>(dst.data()), static_cast<std::streamsize>(n));
    }
    return static_cast<bool>(f);
  };
  auto readU32 = [&](uint32_t& v) {
    unsigned char b[4];
    f.read(reinterpret_cast<char*>(b), 4);
    v = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
        (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
    return static_cast<bool>(f);
  };
  auto readU8 = [&](uint8_t& v) {
    unsigned char b = 0;
    f.read(reinterpret_cast<char*>(&b), 1);
    v = b;
    return static_cast<bool>(f);
  };
  auto readU64 = [&](uint64_t& v) {
    unsigned char b[8];
    f.read(reinterpret_cast<char*>(b), 8);
    v = 0;
    for (int i = 0; i < 8; ++i) {
      v |= static_cast<uint64_t>(b[i]) << (8 * i);
    }
    return static_cast<bool>(f);
  };
  auto readStr = [&](std::string& s) {
    uint32_t len = 0;
    if (!readU32(len)) {
      return false;
    }
    // Bound the length by the bytes still ahead of the stream cursor, not the
    // whole file: len <= file_size can still be larger than what remains after
    // the current position, letting readN resize(len) before failing at EOF.
    const std::streamoff remaining = file_size - static_cast<std::streamoff>(f.tellg());
    if (static_cast<std::streamoff>(len) > remaining) {
      return false;
    }
    s.resize(len);
    if (len > 0) {
      f.read(s.data(), static_cast<std::streamsize>(len));
    }
    return static_cast<bool>(f);
  };

  char magic[4];
  f.read(magic, 4);
  if (!f || std::memcmp(magic, kMagic, 4) != 0) {
    return bad("not a .bfdb file (bad magic)");
  }
  uint32_t format = 0;
  if (!readU32(format)) {
    return bad("truncated .bfdb header");
  }
  if (format != kFormatVersion) {
    return Result<UnifiedData>::Err(
        Error(ErrorCode::kFormatMismatch, WithRebuildHint("incompatible .bfdb format version")));
  }
  uint32_t section_count = 0;
  if (!readU32(section_count) || section_count != kSectionCount) {
    return bad("corrupt .bfdb: unexpected section count");
  }

  UnifiedData out;
  if (!readU32(out.header.cycle) || !readStr(out.header.program_version) ||
      !readStr(out.header.source_loader) || !readStr(out.header.data_dir)) {
    return bad("corrupt .bfdb header");
  }
  uint8_t cap_airway = 0;
  uint8_t cap_alt = 0;
  uint8_t cap_mora = 0;
  uint8_t cap_msa = 0;
  if (!readU8(cap_airway) || !readU8(cap_alt) || !readU8(cap_mora) || !readU8(cap_msa)) {
    return bad("corrupt .bfdb header");
  }
  out.header.capabilities.airway_direction = cap_airway != 0;
  out.header.capabilities.altitude_bands = cap_alt != 0;
  out.header.capabilities.mora_grid = cap_mora != 0;
  out.header.capabilities.msa_sectors = cap_msa != 0;
  uint32_t pool_len = 0;
  if (!readU32(pool_len)) {
    return bad("corrupt .bfdb header");
  }
  uint32_t pool_crc = 0;
  if (!readU32(pool_crc)) {
    return bad("corrupt .bfdb header");
  }
  // Bound by the bytes remaining after the cursor, not the whole file (the pool
  // and every section body still lie ahead, so pool_len can never exceed this).
  const std::streamoff pool_remaining = file_size - static_cast<std::streamoff>(f.tellg());
  if (static_cast<std::streamoff>(pool_len) > pool_remaining) {
    return bad("corrupt .bfdb: pool length exceeds file");
  }

  // Section table (fixed 3 rows). Validate every present section against the
  // file bounds now, so the individual body reads below can trust them.
  struct Row {
    uint32_t type;
    uint32_t crc;  // placed second so off/len stay 8-aligned (no padding)
    uint64_t off, len;
  };
  Row rows[kSectionCount];
  for (uint32_t i = 0; i < section_count; ++i) {
    if (!readU32(rows[i].type) || !readU32(rows[i].crc) || !readU64(rows[i].off) ||
        !readU64(rows[i].len)) {
      return bad("corrupt .bfdb section table");
    }
    // Absent section: off == len == 0. Present: must lie within the file.
    if (rows[i].off != 0 || rows[i].len != 0) {
      if (rows[i].off > static_cast<uint64_t>(file_size) ||
          rows[i].len > static_cast<uint64_t>(file_size) - rows[i].off) {
        return bad("corrupt .bfdb: section reference out of range");
      }
    }
  }

  // Global string pool, read once into memory and shared by every section decode.
  std::vector<uint8_t> pool;
  if (!readN(pool, pool_len)) {
    return bad("truncated .bfdb string pool");
  }
  if (Crc32C::Compute(pool) != pool_crc) {
    return bad("corrupt .bfdb: string pool CRC mismatch");
  }

  // Map section types to rows.
  const Row* graph_row = nullptr;
  const Row* cifp_row = nullptr;
  const Row* detail_row = nullptr;
  for (uint32_t i = 0; i < section_count; ++i) {
    switch (rows[i].type) {
      case kSectionGraph:
        graph_row = &rows[i];
        break;
      case kSectionCifp:
        cifp_row = &rows[i];
        break;
      case kSectionDetail:
        detail_row = &rows[i];
        break;
      default:
        break;  // unknown section type: ignore (forward-compat within a version)
    }
  }
  if (graph_row == nullptr || (graph_row->off == 0 && graph_row->len == 0)) {
    return bad("corrupt .bfdb: missing graph section");
  }

  // A helper to read one section body into a buffer at its offset.
  auto readSection = [&](const Row& row, std::vector<uint8_t>& dst) {
    f.clear();
    f.seekg(static_cast<std::streamoff>(row.off));
    dst.resize(static_cast<size_t>(row.len));
    if (row.len > 0) {
      f.read(reinterpret_cast<char*>(dst.data()), static_cast<std::streamsize>(row.len));
    }
    return static_cast<bool>(f);
  };

  // Graph section (always present): decode against the global pool.
  {
    std::vector<uint8_t> body;
    if (!readSection(*graph_row, body)) {
      return bad("truncated .bfdb graph section");
    }
    if (Crc32C::Compute(body) != graph_row->crc) {
      return bad("corrupt .bfdb: graph section CRC mismatch");
    }
    Result<GraphSnapshot> g = GraphCodec::Decode(body, pool);
    if (!g) {
      return Result<UnifiedData>::Err(std::move(g).error());
    }
    out.graph = std::move(g).value();
  }

  // Detail section (optional): decode against the global pool.
  if (detail_row != nullptr && !(detail_row->off == 0 && detail_row->len == 0)) {
    std::vector<uint8_t> body;
    if (!readSection(*detail_row, body)) {
      return bad("truncated .bfdb detail section");
    }
    if (Crc32C::Compute(body) != detail_row->crc) {
      return bad("corrupt .bfdb: detail section CRC mismatch");
    }
    Result<NavDetailArchive> d = NavDetailCodec::Decode(body, pool);
    if (!d) {
      return Result<UnifiedData>::Err(std::move(d).error());
    }
    out.detail = std::move(d).value();
  }

  // CIFP section (optional): opened on-demand -- the directory is read now, the
  // segments stay on disk and are fetched lazily via a pread handle. The archive
  // needs its own copy of the global pool blob to resolve segment string refs.
  // This is the last use of `pool`, so move it into the archive.
  if (cifp_row != nullptr && !(cifp_row->off == 0 && cifp_row->len == 0)) {
    Result<CifpArchive> c =
        CifpCodec::OpenSection(path, cifp_row->off, cifp_row->len, cifp_row->crc, std::move(pool));
    if (!c) {
      return Result<UnifiedData>::Err(std::move(c).error());
    }
    out.cifp = std::move(c).value();
  }

  return Result<UnifiedData>::Ok(std::move(out));
}

}  // namespace bf
