#include "io/cache/cifp_cache.h"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

#include "io/cache/byte_io.h"

namespace bf {

namespace {

constexpr char kMagic[4] = {'B', 'F', 'C', 'P'};

namespace fs = std::filesystem;

// --- One airport's CIFP data, serialized as a self-contained segment. ---
//
// A segment has its own trailing string pool, so it can be deserialized in
// isolation given only its bytes (no dependence on other segments). This is
// what makes on-demand, per-airport loading possible.

std::string SerializeSegment(const CifpData& data) {
  StringPool pool;
  std::string body;
  ByteWriter w(body);

  auto ref = [&](const std::string& s) {
    const auto r = pool.Add(s);
    w.U32(r.first);
    w.U32(r.second);
  };

  w.U32(static_cast<uint32_t>(data.procedures.size()));
  for (const Procedure& p : data.procedures) {
    w.U8(static_cast<uint8_t>(p.type));
    w.I32(p.route_type);
    ref(p.name);
    ref(p.transition_ident);
    ref(p.runway);
    w.U32(static_cast<uint32_t>(p.legs.size()));
    for (const ProcedureLeg& leg : p.legs) {
      ref(leg.fix.ident);
      ref(leg.fix.region);
      w.U8(static_cast<uint8_t>(leg.path_term));
      w.F64(leg.course_deg);
      w.F64(leg.distance_nm);
      w.U8(static_cast<uint8_t>(leg.alt.kind));
      w.I32(leg.alt.alt1_ft);
      w.I32(leg.alt.alt2_ft);
    }
  }
  w.U32(static_cast<uint32_t>(data.runways.size()));
  for (const Runway& rwy : data.runways) {
    ref(rwy.ident);
    w.F64(rwy.threshold.latitude);
    w.F64(rwy.threshold.longitude);
    w.I32(rwy.elevation_ft);
  }

  // Segment layout: [pool_len][pool blob][body]. The pool precedes the body so
  // the reader can resolve references while streaming the body.
  std::string out;
  ByteWriter hw(out);
  hw.U32(static_cast<uint32_t>(pool.blob().size()));
  out.append(pool.blob());
  out.append(body);
  return out;
}

std::optional<CifpData> DeserializeSegment(const std::string& bytes) {
  ByteReader r(bytes.data(), bytes.size());
  const uint32_t pool_len = r.U32();
  if (!r.ok() || pool_len > r.remaining()) {
    return std::nullopt;
  }
  const std::string blob = bytes.substr(sizeof(uint32_t), pool_len);
  // Advance the reader past the pool to the body.
  ByteReader br(bytes.data() + sizeof(uint32_t) + pool_len,
                bytes.size() - sizeof(uint32_t) - pool_len);

  bool refs_ok = true;
  auto ref = [&](std::string& s) {
    const uint32_t off = br.U32();
    const uint32_t len = br.U32();
    s = ResolveRef(blob, off, len, refs_ok);
  };

  CifpData data;
  const uint32_t proc_count = br.U32();
  if (!br.ok() || proc_count > br.remaining()) {
    return std::nullopt;
  }
  data.procedures.resize(proc_count);
  for (uint32_t i = 0; i < proc_count; ++i) {
    Procedure& p = data.procedures[i];
    p.type = static_cast<ProcedureType>(br.U8());
    p.route_type = br.I32();
    ref(p.name);
    ref(p.transition_ident);
    ref(p.runway);
    const uint32_t leg_count = br.U32();
    if (!br.ok() || leg_count > br.remaining()) {
      return std::nullopt;
    }
    p.legs.resize(leg_count);
    for (uint32_t j = 0; j < leg_count; ++j) {
      ProcedureLeg& leg = p.legs[j];
      ref(leg.fix.ident);
      ref(leg.fix.region);
      leg.path_term = static_cast<PathTerminator>(br.U8());
      leg.course_deg = br.F64();
      leg.distance_nm = br.F64();
      leg.alt.kind = static_cast<AltConstraintKind>(br.U8());
      leg.alt.alt1_ft = br.I32();
      leg.alt.alt2_ft = br.I32();
    }
  }
  const uint32_t rwy_count = br.U32();
  if (!br.ok() || rwy_count > br.remaining()) {
    return std::nullopt;
  }
  data.runways.resize(rwy_count);
  for (uint32_t i = 0; i < rwy_count; ++i) {
    Runway& rwy = data.runways[i];
    ref(rwy.ident);
    rwy.threshold.latitude = br.F64();
    rwy.threshold.longitude = br.F64();
    rwy.elevation_ft = br.I32();
  }

  if (!br.ok() || !refs_ok) {
    return std::nullopt;
  }
  return data;
}

}  // namespace

Result<uint32_t> CifpCache::Build(const std::string& data_dir, const std::string& out_path,
                                  const std::string& source_loader, uint32_t cycle, uint32_t build,
                                  const std::string& program_semver) {
  const fs::path cifp_dir = fs::path(data_dir) / "CIFP";
  std::error_code ec;
  if (!fs::is_directory(cifp_dir, ec)) {
    return Result<uint32_t>::Err(
        Error(ErrorCode::kDataMissing, "no CIFP directory under " + data_dir));
  }

  // Directory entries and segment bodies are accumulated first; offsets are
  // fixed up once the header + directory size is known.
  struct Entry {
    std::string icao;
    std::string body;
  };
  std::vector<Entry> entries;
  for (const fs::directory_entry& de : fs::directory_iterator(cifp_dir, ec)) {
    if (!de.is_regular_file() || de.path().extension() != ".dat") {
      continue;
    }
    const std::string icao = de.path().stem().string();
    Result<CifpData> parsed = CifpParser::Parse(de.path().string());
    if (!parsed) {
      continue;  // skip unreadable file; not fatal for the archive as a whole
    }
    entries.push_back({icao, SerializeSegment(parsed.value())});
  }

  // Header + directory string area.
  StringPool dir_pool;
  std::string header;
  ByteWriter hw(header);
  header.append(kMagic, 4);
  hw.U32(kFormatVersion);
  auto write_inline = [&](const std::string& s) {
    hw.U32(static_cast<uint32_t>(s.size()));
    header.append(s);
  };
  write_inline(program_semver);
  write_inline(source_loader);
  hw.U32(cycle);
  hw.U32(build);
  hw.U32(static_cast<uint32_t>(entries.size()));

  // Directory: each entry is icao_ref(off,len) + seg_offset(u64) + seg_len(u32).
  // Segment offsets are absolute from the file start, so compute the layout:
  // [header][directory][dir string pool][segments].
  std::string directory;
  ByteWriter dw(directory);
  const size_t dir_entry_size = 4 + 4 + 8 + 4;  // icao off,len + offset + len
  // Provisional sizes to locate the first segment. The directory string pool
  // holds the ICAO codes.
  std::string dir_pool_blob;
  {
    // First pass: build the ICAO pool so its size is known.
    for (const Entry& e : entries) {
      dir_pool.Add(e.icao);
    }
    dir_pool_blob = dir_pool.blob();
  }
  const size_t segments_start =
      header.size() + entries.size() * dir_entry_size + 4 /*dir_pool_len*/ + dir_pool_blob.size();

  // Second pass: emit directory rows with absolute segment offsets.
  uint64_t running = segments_start;
  size_t pool_cursor = 0;
  for (const Entry& e : entries) {
    dw.U32(static_cast<uint32_t>(pool_cursor));
    dw.U32(static_cast<uint32_t>(e.icao.size()));
    pool_cursor += e.icao.size();
    dw.U64(running);
    dw.U32(static_cast<uint32_t>(e.body.size()));
    running += e.body.size();
  }

  std::string out;
  out.append(header);
  out.append(directory);
  ByteWriter pw(out);
  pw.U32(static_cast<uint32_t>(dir_pool_blob.size()));
  out.append(dir_pool_blob);
  for (const Entry& e : entries) {
    out.append(e.body);
  }

  std::ofstream f(out_path, std::ios::binary | std::ios::trunc);
  if (!f.is_open()) {
    return Result<uint32_t>::Err(
        Error(ErrorCode::kDataMissing, "cannot open CIFP cache for writing: " + out_path));
  }
  f.write(out.data(), static_cast<std::streamsize>(out.size()));
  if (!f) {
    return Result<uint32_t>::Err(Error(ErrorCode::kParseError, "failed writing " + out_path));
  }
  return Result<uint32_t>::Ok(static_cast<uint32_t>(entries.size()));
}

Result<CifpArchive> CifpCache::Open(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kDataMissing, "cannot open CIFP cache: " + path));
  }
  // Read the whole header+directory region. We don't know its exact length up
  // front, so read the entire file's leading portion up to the segment area is
  // awkward; instead read the full file header lazily by reading enough bytes.
  // The directory is small relative to segments, so read the entire file's
  // prefix by first reading a generous chunk, then re-reading if needed. Simpler
  // and robust: read the full file into memory only for the header+directory by
  // reading incrementally. Here we read the whole file prefix by streaming.
  auto bad = [&](const char* why) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kDataMissing, std::string(why) + "; run bf build to regenerate"));
  };

  // Read a bounded prefix large enough for header + directory. We first read the
  // fixed header fields to learn airport_count, then the directory precisely.
  char magic[4];
  f.read(magic, 4);
  if (!f || std::memcmp(magic, kMagic, 4) != 0) {
    return bad("not a CIFP cache (bad magic)");
  }

  // Helper to read fixed-width little-endian ints directly from the stream.
  auto readU32 = [&](uint32_t& v) {
    unsigned char b[4];
    f.read(reinterpret_cast<char*>(b), 4);
    v = static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
        (static_cast<uint32_t>(b[2]) << 16) | (static_cast<uint32_t>(b[3]) << 24);
    return static_cast<bool>(f);
  };
  auto readInline = [&](std::string& s) {
    uint32_t len = 0;
    if (!readU32(len)) {
      return false;
    }
    s.resize(len);
    if (len > 0) {
      f.read(s.data(), len);
    }
    return static_cast<bool>(f);
  };

  CifpArchive archive;
  archive.path_ = path;
  uint32_t format = 0;
  if (!readU32(format)) {
    return bad("truncated CIFP cache header");
  }
  if (format != kFormatVersion) {
    return bad("incompatible CIFP cache format version");
  }
  if (!readInline(archive.program_semver_) || !readInline(archive.source_loader_)) {
    return bad("corrupt CIFP cache header");
  }
  uint32_t airport_count = 0;
  if (!readU32(archive.cycle_) || !readU32(archive.build_) || !readU32(airport_count)) {
    return bad("corrupt CIFP cache header");
  }

  // Read the directory rows, then the directory string pool.
  struct Row {
    uint32_t icao_off, icao_len;
    uint64_t seg_off;
    uint32_t seg_len;
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
  std::vector<Row> rows(airport_count);
  for (uint32_t i = 0; i < airport_count; ++i) {
    if (!readU32(rows[i].icao_off) || !readU32(rows[i].icao_len) || !readU64(rows[i].seg_off) ||
        !readU32(rows[i].seg_len)) {
      return bad("corrupt CIFP cache directory");
    }
  }
  uint32_t dir_pool_len = 0;
  if (!readU32(dir_pool_len)) {
    return bad("corrupt CIFP cache directory");
  }
  std::string dir_pool(dir_pool_len, '\0');
  if (dir_pool_len > 0) {
    f.read(dir_pool.data(), dir_pool_len);
  }
  if (!f) {
    return bad("truncated CIFP cache directory");
  }

  archive.index_.reserve(airport_count);
  for (const Row& row : rows) {
    if (static_cast<size_t>(row.icao_off) + row.icao_len > dir_pool.size()) {
      return bad("corrupt CIFP cache: ICAO reference out of range");
    }
    std::string icao = dir_pool.substr(row.icao_off, row.icao_len);
    archive.index_.emplace(std::move(icao), std::make_pair(row.seg_off, row.seg_len));
  }
  return Result<CifpArchive>::Ok(std::move(archive));
}

std::optional<CifpData> CifpArchive::Fetch(const std::string& icao) const {
  auto it = index_.find(icao);
  if (it == index_.end()) {
    return std::nullopt;
  }
  const uint64_t offset = it->second.first;
  const uint32_t length = it->second.second;
  // Independent ifstream per call: no shared mutable state, so concurrent
  // fetches for different airports are race-free (contract B).
  std::ifstream f(path_, std::ios::binary);
  if (!f.is_open()) {
    return std::nullopt;
  }
  f.seekg(static_cast<std::streamoff>(offset));
  std::string bytes(length, '\0');
  f.read(bytes.data(), length);
  if (!f) {
    return std::nullopt;
  }
  return DeserializeSegment(bytes);
}

}  // namespace bf
