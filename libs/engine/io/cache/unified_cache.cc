// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/unified_cache.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <fstream>
#include <random>
#include <span>
#include <system_error>
#include <utility>
#include <vector>

#include "io/cache/byte_io.h"
#include "io/cache/cache_messages.h"
#include "io/cache/cifp_codec.h"
#include "io/cache/crc32c.h"
#include "io/cache/graph_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"
#include "io/cache/pread_file.h"

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
constexpr size_t kHeaderCrcSize = 4;  // CRC-32C of [header][section table], right after the table

// A generous bound on the header prefix (magic + fixed U32s + three inline
// provenance strings). Real provenance strings are a few dozen bytes and a data
// dir is a filesystem path, so 8 KiB covers the whole prefix in one read; a
// string length pointing past what was read means the file is corrupt.
constexpr size_t kMaxHeaderPrefix = 8192;

// ofstream speaks char*; the cache layer's byte buffers are
// std::vector<uint8_t>. This helper centralizes the reinterpret_cast so every
// write site stays byte-typed.
void WriteAll(std::ostream& f, std::span<const uint8_t> b) {
  f.write(reinterpret_cast<const char*>(b.data()), static_cast<std::streamsize>(b.size()));
}

// A sibling of `path` to write the new cache into before renaming it over
// `path`. Same directory, so the rename never crosses filesystems; a random
// suffix, so two concurrent builds of one path do not write into the same
// file; and a ".tmp" extension, so a directory scan for caches skips it.
std::string TempPathFor(const std::string& path) {
  std::random_device rd;
  const uint64_t tag = (static_cast<uint64_t>(rd()) << 32) | rd();
  return std::format("{}.{:016x}.tmp", path, tag);
}

// Replace `to` with `from` in one step: a reader never sees a missing or
// half-written `to`, and one that already has `to` open keeps the old file.
Result<void> ReplaceFile(const std::string& from, const std::string& to) {
  namespace fs = std::filesystem;
#if defined(_WIN32) && defined(FILE_RENAME_FLAG_POSIX_SEMANTICS)
  // MoveFileEx (what std::filesystem::rename uses) refuses to replace a file
  // that some process still has open -- here, any running server reading the
  // old cache. A POSIX-semantics rename (Windows 10 1607+, NTFS) unlinks the
  // old name instead, as rename(2) does; the readers' FILE_SHARE_DELETE (see
  // PreadFile) is what permits it. Where it is unsupported, fall through to
  // std::filesystem::rename.
  std::error_code abs_ec;
  const std::wstring target = fs::absolute(fs::path(to), abs_ec).wstring();
  const HANDLE h = abs_ec ? INVALID_HANDLE_VALUE
                          : ::CreateFileW(fs::path(from).c_str(), DELETE,
                                          FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                                          nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (h != INVALID_HANDLE_VALUE) {
    // FILE_RENAME_INFO ends in a flexible FileName array; size the buffer for
    // the target name and align it for the struct.
    const size_t name_bytes = target.size() * sizeof(wchar_t);
    std::vector<uint64_t> buf((sizeof(FILE_RENAME_INFO) + name_bytes + sizeof(uint64_t) - 1) /
                              sizeof(uint64_t));
    auto* info = reinterpret_cast<FILE_RENAME_INFO*>(buf.data());
    info->Flags = FILE_RENAME_FLAG_REPLACE_IF_EXISTS | FILE_RENAME_FLAG_POSIX_SEMANTICS;
    info->RootDirectory = nullptr;
    info->FileNameLength = static_cast<DWORD>(name_bytes);
    std::memcpy(info->FileName, target.data(), name_bytes);
    const BOOL renamed = ::SetFileInformationByHandle(
        h, FileRenameInfoEx, info, static_cast<DWORD>(buf.size() * sizeof(uint64_t)));
    ::CloseHandle(h);
    if (renamed) {
      return Result<void>::Ok();
    }
  }
#endif
  std::error_code ec;
  fs::rename(fs::path(from), fs::path(to), ec);
  if (ec) {
    return Result<void>::Err(
        Error(ErrorCode::kSerializationError,
              std::format("cannot replace .bfdb: {} ({}); the previous file, if any, is unchanged",
                          to, ec.message())));
  }
  return Result<void>::Ok();
}

// Sequential reads through a PreadFile, for walking the container header. The
// first failed or out-of-range read sticks (like a stream's failbit), so a run
// of reads can be checked once at the end.
class FileCursor {
 public:
  FileCursor(const PreadFile& file, uint64_t size) : file_(file), size_(size) {}

  bool Read(std::span<uint8_t> dst) {
    if (!ok_ || dst.size() > remaining() || !file_.ReadAt(dst, pos_)) {
      ok_ = false;
      return false;
    }
    pos_ += dst.size();
    return true;
  }

  uint64_t pos() const { return pos_; }
  uint64_t remaining() const { return size_ - pos_; }

 private:
  const PreadFile& file_;
  uint64_t size_;
  uint64_t pos_ = 0;
  bool ok_ = true;
};

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

  // Sections begin after [header][section table][header CRC][pool]. Compute
  // each present section's absolute file offset in table order (graph, cifp,
  // detail).
  const uint64_t sections_start = header.size() +
                                  static_cast<uint64_t>(kSectionCount) * kSectionRowSize +
                                  kHeaderCrcSize + pool_blob.size();
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

  // The header CRC covers every byte before it -- the fixed header, provenance,
  // capabilities, pool length/CRC, and the section table -- so a bit flip in a
  // cycle, a capability flag, or a section row's type/offset/length is caught at
  // Open instead of silently cataloguing the wrong cycle or dropping a section.
  std::vector<uint8_t> header_crc;
  {
    ByteWriter cw(header_crc);
    cw.U32(Crc32C::Update(Crc32C::Compute(header), table));
  }

  // Write a temporary file beside `path` and rename it over `path` only once
  // every byte is written. Truncating `path` in place would destroy a good
  // cache whenever the rebuild fails part-way, and would change the bytes under
  // a server that has the old file open; a rename leaves the old file intact
  // for both.
  const std::string tmp_path = TempPathFor(path);
  auto discard_tmp = [&] {
    std::error_code ignored;
    std::filesystem::remove(tmp_path, ignored);
  };
  {
    std::ofstream f(tmp_path, std::ios::binary | std::ios::trunc);
    if (!f.is_open()) {
      return Result<void>::Err(Error(ErrorCode::kDataMissing,
                                     std::format("cannot open .bfdb for writing: {}", tmp_path)));
    }
    WriteAll(f, header);
    WriteAll(f, table);
    WriteAll(f, header_crc);
    WriteAll(f, pool_blob);
    WriteAll(f, graph_body);
    if (has_cifp) {
      WriteAll(f, cifp_body);
    }
    if (has_detail) {
      WriteAll(f, detail_body);
    }
    // close() flushes the buffered tail; a failure there only shows after it.
    f.close();
    if (f.fail()) {
      discard_tmp();
      return Result<void>::Err(
          Error(ErrorCode::kSerializationError, std::format("failed writing .bfdb: {}", path)));
    }
  }
  Result<void> replaced = ReplaceFile(tmp_path, path);
  if (!replaced) {
    discard_tmp();
  }
  return replaced;
}

Result<UnifiedHeader> UnifiedCache::ReadHeader(const std::string& path) {
  const PreadFile file(path);
  const std::optional<uint64_t> file_size = file.is_open() ? file.Size() : std::nullopt;
  if (!file_size) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kDataMissing, std::format("cannot open .bfdb: {}", path)));
  }
  // The whole prefix in one read, or the whole file if it is smaller.
  std::vector<uint8_t> buf(static_cast<size_t>(std::min<uint64_t>(*file_size, kMaxHeaderPrefix)));
  if (!file.ReadAt(buf, 0)) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kDataMissing, std::format("cannot read .bfdb: {}", path)));
  }
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
  r.U32();  // pool_len
  r.U32();  // pool_crc
  uint8_t table[kSectionCount * kSectionRowSize];
  r.Bytes(table, sizeof(table));  // covered by the header CRC, otherwise unused here
  // Bytes [0, crc_at) are the magic plus everything the reader has consumed.
  const size_t crc_at = buf.size() - r.remaining();
  const uint32_t stored_crc = r.U32();
  if (!r.ok()) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kCacheCorrupt, WithRebuildHint("corrupt .bfdb header")));
  }
  // Verify the header CRC so a catalogue scan (BfdbInventory) cannot list a
  // bit-flipped cycle or provenance as if it were real.
  if (Crc32C::Compute(std::span<const uint8_t>(buf).subspan(0, crc_at)) != stored_crc) {
    return Result<UnifiedHeader>::Err(
        Error(ErrorCode::kCacheCorrupt, WithRebuildHint("corrupt .bfdb: header CRC mismatch")));
  }
  return Result<UnifiedHeader>::Ok(std::move(header));
}

Result<UnifiedData> UnifiedCache::Open(const std::string& path) {
  // One handle serves every read, the CIFP archive's lazy segment fetches
  // included: were the path replaced by a rebuild part-way through, a second
  // open would see the new file at offsets taken from the old one's header.
  PreadFile file(path);
  const std::optional<uint64_t> size = file.is_open() ? file.Size() : std::nullopt;
  if (!size) {
    return Result<UnifiedData>::Err(
        Error(ErrorCode::kDataMissing, std::format("cannot open .bfdb: {}", path)));
  }
  const uint64_t file_size = *size;

  auto bad = [&](const char* why) {
    return Result<UnifiedData>::Err(Error(ErrorCode::kCacheCorrupt, WithRebuildHint(why)));
  };

  if (file_size < 4) {
    return bad("truncated .bfdb");
  }

  // Read the fixed header + provenance strings + section table + global pool.
  // These are near the file start; the (large) section bodies are read
  // individually afterward, so this initial read stays small. Every read is
  // bounded by the bytes left in the file, so a corrupt length fails before it
  // can size an allocation.
  FileCursor cur(file, file_size);
  auto readN = [&](std::vector<uint8_t>& dst, uint64_t n) {
    if (n > cur.remaining()) {
      return false;
    }
    dst.resize(static_cast<size_t>(n));
    return cur.Read(dst);
  };
  auto readU32 = [&](uint32_t& v) {
    uint8_t b[4] = {};
    const bool ok = cur.Read(b);
    v = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
        (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
    return ok;
  };
  auto readU8 = [&](uint8_t& v) {
    uint8_t b[1] = {};
    const bool ok = cur.Read(b);
    v = b[0];
    return ok;
  };
  auto readU64 = [&](uint64_t& v) {
    uint8_t b[8] = {};
    const bool ok = cur.Read(b);
    v = 0;
    for (int i = 0; i < 8; ++i) {
      v |= static_cast<uint64_t>(b[i]) << (8 * i);
    }
    return ok;
  };
  auto readStr = [&](std::string& s) {
    uint32_t len = 0;
    if (!readU32(len) || len > cur.remaining()) {
      return false;
    }
    s.resize(len);
    return cur.Read(std::span<uint8_t>(reinterpret_cast<uint8_t*>(s.data()), s.size()));
  };

  uint8_t magic[4] = {};
  if (!cur.Read(magic) || std::memcmp(magic, kMagic, 4) != 0) {
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
  if (pool_len > cur.remaining()) {
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
      if (rows[i].off > file_size || rows[i].len > file_size - rows[i].off) {
        return bad("corrupt .bfdb: section reference out of range");
      }
    }
  }

  // Header CRC over every byte read so far ([header][section table]): re-read
  // that prefix (a few hundred bytes) and compare.
  const uint64_t header_end = cur.pos();
  uint32_t header_crc = 0;
  if (!readU32(header_crc)) {
    return bad("truncated .bfdb header");
  }
  {
    std::vector<uint8_t> prefix(static_cast<size_t>(header_end));
    if (!file.ReadAt(prefix, 0)) {
      return bad("truncated .bfdb header");
    }
    if (Crc32C::Compute(prefix) != header_crc) {
      return bad("corrupt .bfdb: header CRC mismatch");
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

  // Map section types to rows. A format version fixes the section set, so each
  // known type appears exactly once; an unknown or repeated type is corruption
  // (the header CRC already makes it unlikely), not something to skip past.
  const Row* graph_row = nullptr;
  const Row* cifp_row = nullptr;
  const Row* detail_row = nullptr;
  for (uint32_t i = 0; i < section_count; ++i) {
    const Row** slot = nullptr;
    switch (rows[i].type) {
      case kSectionGraph:
        slot = &graph_row;
        break;
      case kSectionCifp:
        slot = &cifp_row;
        break;
      case kSectionDetail:
        slot = &detail_row;
        break;
      default:
        return bad("corrupt .bfdb: unknown section type");
    }
    if (*slot != nullptr) {
      return bad("corrupt .bfdb: duplicate section type");
    }
    *slot = &rows[i];
  }
  if (graph_row == nullptr || (graph_row->off == 0 && graph_row->len == 0)) {
    return bad("corrupt .bfdb: missing graph section");
  }

  // A helper to read one section body into a buffer at its offset (already
  // checked against the file size above).
  auto readSection = [&](const Row& row, std::vector<uint8_t>& dst) {
    dst.resize(static_cast<size_t>(row.len));
    return file.ReadAt(dst, row.off);
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
    Result<CifpArchive> c = CifpCodec::OpenSection(std::move(file), cifp_row->off, cifp_row->len,
                                                   cifp_row->crc, std::move(pool));
    if (!c) {
      return Result<UnifiedData>::Err(std::move(c).error());
    }
    out.cifp = std::move(c).value();
  }

  return Result<UnifiedData>::Ok(std::move(out));
}

}  // namespace bf
