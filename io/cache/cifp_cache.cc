#include "io/cache/cifp_cache.h"

#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

#include "io/cache/byte_io.h"

namespace bf {

namespace {

constexpr char kMagic[4] = {'B', 'F', 'C', 'P'};

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

std::optional<CifpData> DeserializeSegment(const char* data, size_t size) {
  ByteReader r(data, size);
  const uint32_t pool_len = r.U32();
  if (!r.ok() || pool_len > r.remaining()) {
    return std::nullopt;
  }
  const char* blob = data + sizeof(uint32_t);
  // Advance the reader past the pool to the body.
  ByteReader br(data + sizeof(uint32_t) + pool_len, size - sizeof(uint32_t) - pool_len);

  bool refs_ok = true;
  auto ref = [&](std::string& s) {
    const uint32_t off = br.U32();
    const uint32_t len = br.U32();
    s = ResolveRef(blob, pool_len, off, len, refs_ok);
  };

  // Minimum on-disk bytes per record, used to reject an absurd count before
  // resizing (ByteReader still guards the actual reads, but this stops a forged
  // count from forcing a huge allocation): a procedure is >= 33 B (type 1 +
  // route_type 4 + 3 string refs 24 + leg count 4), a leg >= 42 B (2 refs 16 +
  // path_term 1 + course 8 + distance 8 + alt kind 1 + alt1 4 + alt2 4), a
  // runway >= 28 B (ident ref 8 + lat 8 + lon 8 + elevation 4).
  auto count_fits = [&](uint32_t count, size_t per_record) {
    return static_cast<size_t>(count) <= br.remaining() / per_record;
  };

  CifpData data_out;
  const uint32_t proc_count = br.U32();
  if (!br.ok() || !count_fits(proc_count, 33)) {
    return std::nullopt;
  }
  data_out.procedures.resize(proc_count);
  for (uint32_t i = 0; i < proc_count; ++i) {
    Procedure& p = data_out.procedures[i];
    p.type = static_cast<ProcedureType>(br.U8());
    p.route_type = br.I32();
    ref(p.name);
    ref(p.transition_ident);
    ref(p.runway);
    const uint32_t leg_count = br.U32();
    if (!br.ok() || !count_fits(leg_count, 42)) {
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
  if (!br.ok() || !count_fits(rwy_count, 28)) {
    return std::nullopt;
  }
  data_out.runways.resize(rwy_count);
  for (uint32_t i = 0; i < rwy_count; ++i) {
    Runway& rwy = data_out.runways[i];
    ref(rwy.ident);
    rwy.threshold.latitude = br.F64();
    rwy.threshold.longitude = br.F64();
    rwy.elevation_ft = br.I32();
  }

  if (!br.ok() || !refs_ok) {
    return std::nullopt;
  }
  return data_out;
}

}  // namespace

Result<uint32_t> CifpCache::Build(const std::vector<std::pair<std::string, CifpData>>& procedures,
                                  const std::string& out_path, const std::string& source_loader,
                                  uint32_t cycle, uint32_t build,
                                  const std::string& program_semver) {
  // Directory entries and segment bodies are accumulated first; offsets are
  // fixed up once the header + directory size is known. The caller supplies
  // already-parsed (ICAO, CifpData) pairs, so this is source-agnostic: it never
  // touches any data source's on-disk layout.
  struct Entry {
    std::string icao;
    std::string body;
  };
  std::vector<Entry> entries;
  entries.reserve(procedures.size());
  for (const auto& [icao, data] : procedures) {
    entries.push_back({icao, SerializeSegment(data)});
  }

  // Header + directory string area.
  StringPool dir_pool;
  std::string header;
  ByteWriter hw(header);
  header.append(kMagic, 4);
  hw.U32(kFormatVersion);
  hw.Str(program_semver);
  hw.Str(source_loader);
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
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kDataMissing, "cannot open CIFP cache: " + path));
  }
  // Total size, used as an allocation ceiling below: every length/count read
  // from the (possibly corrupt) header or directory must fit within the file,
  // so a forged field cannot trigger a huge resize and a bad_alloc that would
  // bypass Result.
  const std::streamoff file_size = f.tellg();
  f.seekg(0);

  auto bad = [&](const char* why) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kDataMissing, std::string(why) + "; run bf build to regenerate"));
  };

  // Read the fixed header to learn the entry count, then read the directory rows
  // and string pool exactly; the per-airport segments stay on disk and are read
  // lazily by Fetch. The directory is small relative to the segments.
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
    // Bound the length by the file size before resizing (contrast with the
    // earlier code, which resized on an untrusted length).
    if (static_cast<std::streamoff>(len) > file_size) {
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
  // A directory row is 24 bytes on disk (icao off+len 8, seg offset 8, seg len
  // 4, ... = 4+4+8+4); reject a count that could not fit before allocating.
  constexpr std::streamoff kRowBytes = 4 + 4 + 8 + 4;
  if (static_cast<std::streamoff>(airport_count) > file_size / kRowBytes) {
    return bad("corrupt CIFP cache: directory count exceeds file size");
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
  if (static_cast<std::streamoff>(dir_pool_len) > file_size) {
    return bad("corrupt CIFP cache: directory pool exceeds file size");
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
    // Cross-check each segment against the file bounds now, so Fetch/FetchAll can
    // allocate seg_len without re-validating (offset is u64, len is u32; the sum
    // cannot overflow size_t on a 64-bit target). A corrupt directory makes the
    // whole cache untrustworthy, so reject rather than skip the row.
    if (row.seg_off > static_cast<uint64_t>(file_size) ||
        row.seg_len > static_cast<uint64_t>(file_size) - row.seg_off) {
      return bad("corrupt CIFP cache: segment reference out of range");
    }
    std::string icao = dir_pool.substr(row.icao_off, row.icao_len);
    archive.index_.emplace(std::move(icao), std::make_pair(row.seg_off, row.seg_len));
  }
  return Result<CifpArchive>::Ok(std::move(archive));
}

std::unordered_map<std::string, CifpData> CifpArchive::FetchAll() const {
  std::unordered_map<std::string, CifpData> out;
  // Read the whole file once, then slice each segment from memory.
  std::ifstream f(path_, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    return out;
  }
  const std::streamsize size = f.tellg();
  if (size <= 0) {
    return out;
  }
  std::string buf(static_cast<size_t>(size), '\0');
  f.seekg(0);
  f.read(buf.data(), size);
  if (!f) {
    return out;
  }
  out.reserve(index_.size());
  for (const auto& entry : index_) {
    const uint64_t offset = entry.second.first;
    const uint32_t length = entry.second.second;
    if (offset + length > buf.size()) {
      continue;
    }
    // Deserialize directly from the in-memory file buffer, no per-segment copy.
    std::optional<CifpData> data = DeserializeSegment(buf.data() + offset, length);
    if (data.has_value()) {
      out.emplace(entry.first, std::move(*data));
    }
  }
  return out;
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
  return DeserializeSegment(bytes.data(), bytes.size());
}

}  // namespace bf
