#include "io/cache/cifp_codec.h"

#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

#include "io/cache/byte_io.h"

namespace bf {

namespace {

// Each of these enums is serialized as a single U8 in a segment; guard that the
// last enumerator still fits, so extending an enum past 255 fails to compile
// rather than silently truncating on write.
static_assert(static_cast<int>(ProcedureType::kApproach) < 256, "ProcedureType exceeds U8");
static_assert(static_cast<int>(PathTerminator::kUnknown) < 256, "PathTerminator exceeds U8");
static_assert(static_cast<int>(AltConstraintKind::kBetween) < 256, "AltConstraintKind exceeds U8");

// --- One airport's CIFP data, serialized as a bare segment body. ---
//
// Unlike the old standalone CIFP cache, a segment has NO per-segment string
// pool: every string reference points into the container's single global pool,
// so the same fix/ICAO stored across many airports is deduplicated once. A
// segment therefore cannot be deserialized in isolation -- it needs the global
// pool blob, which the CifpArchive holds in memory.

std::string SerializeSegment(const CifpData& data, StringPool& pool) {
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
      // fix is a FixedIdent; serialize its two parts as pool refs, byte-identical
      // to the former Ident layout (idents <=5 chars => SSO, no heap).
      ref(std::string(leg.fix.IdentView()));
      ref(std::string(leg.fix.RegionView()));
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
  return body;
}

// Deserialize a bare segment body, resolving string references against the
// global pool blob (`pool`/`pool_len`).
std::optional<CifpData> DeserializeSegment(const char* data, size_t size, const char* pool,
                                           size_t pool_len) {
  ByteReader br(data, size);

  bool refs_ok = true;
  auto ref = [&](std::string& s) {
    const uint32_t off = br.U32();
    const uint32_t len = br.U32();
    s = ResolveRef(pool, pool_len, off, len, refs_ok);
  };
  // Same read, returning the resolved string by value -- used for the fix, which
  // is a FixedIdent (cannot bind to std::string&) built via FromParts.
  auto read_ref = [&]() -> std::string {
    const uint32_t off = br.U32();
    const uint32_t len = br.U32();
    return ResolveRef(pool, pool_len, off, len, refs_ok);
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
      // Two pool refs (ident, region) -> FixedIdent, order matching Encode.
      const std::string fix_ident = read_ref();
      const std::string fix_region = read_ref();
      leg.fix = FixedIdent::FromParts(fix_ident, fix_region);
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

constexpr size_t kDirEntrySize = 4 + 4 + 8 + 4;  // icao off,len + seg offset + seg len

}  // namespace

Result<uint32_t> CifpCodec::Encode(const std::vector<std::pair<std::string, CifpData>>& procedures,
                                   ByteWriter& w, StringPool& pool) {
  // Serialize each airport's segment body first (interning strings into the
  // shared global pool). Directory rows carry each segment's offset RELATIVE to
  // the CIFP section start, so the whole section is assembled into a local buffer
  // and then appended to the shared writer -- offsets computed here do not depend
  // on how many bytes preceding sections already wrote to `w`.
  struct Entry {
    const std::string* icao;
    std::string body;
  };
  std::vector<Entry> entries;
  entries.reserve(procedures.size());
  for (const auto& [icao, data] : procedures) {
    entries.push_back({&icao, SerializeSegment(data, pool)});
  }

  // Intern every ICAO into the shared pool, capturing each one's ref.
  std::vector<std::pair<uint32_t, uint32_t>> icao_refs;
  icao_refs.reserve(entries.size());
  for (const Entry& e : entries) {
    icao_refs.push_back(pool.Add(*e.icao));
  }

  // Section layout: [airport_count][directory][segments]. Segments begin right
  // after the fixed count and directory.
  const uint64_t segments_start =
      4 /*airport_count*/ + static_cast<uint64_t>(entries.size()) * kDirEntrySize;

  std::string section;
  ByteWriter sw(section);
  sw.U32(static_cast<uint32_t>(entries.size()));
  uint64_t running = segments_start;
  for (size_t i = 0; i < entries.size(); ++i) {
    sw.U32(icao_refs[i].first);
    sw.U32(icao_refs[i].second);
    sw.U64(running);  // relative to CIFP section start
    sw.U32(static_cast<uint32_t>(entries[i].body.size()));
    running += entries[i].body.size();
  }
  for (const Entry& e : entries) {
    section.append(e.body);
  }

  w.Bytes(section.data(), section.size());
  return Result<uint32_t>::Ok(static_cast<uint32_t>(entries.size()));
}

Result<CifpArchive> CifpCodec::OpenSection(const std::string& path, uint64_t section_offset,
                                           uint64_t section_length, std::string pool_blob) {
  auto bad = [&](const char* why) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kCacheCorrupt, std::string(why) + "; run bf build to regenerate"));
  };

  // Open a positional-read handle on the unified file and read the CIFP section's
  // directory. The per-airport segments stay on disk and are fetched lazily.
  CifpArchive archive;
  archive.file_ = PreadFile(path);
  if (!archive.file_.is_open()) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kDataMissing, "cannot open .bfdb for CIFP segment reads: " + path));
  }
  archive.pool_ = std::move(pool_blob);

  if (section_length < 4) {
    return bad("corrupt CIFP section: too short");
  }
  // Read airport_count.
  char count_buf[4];
  if (!archive.file_.ReadAt(count_buf, 4, section_offset)) {
    return bad("truncated CIFP section directory");
  }
  ByteReader cr(count_buf, 4);
  const uint32_t airport_count = cr.U32();
  // A directory row is 20 bytes; reject a count that could not fit in the section
  // before allocating.
  if (static_cast<uint64_t>(airport_count) > (section_length - 4) / kDirEntrySize) {
    return bad("corrupt CIFP section: directory count exceeds section size");
  }

  // Read the whole directory region in one positional read.
  const size_t dir_bytes = static_cast<size_t>(airport_count) * kDirEntrySize;
  std::string dir(dir_bytes, '\0');
  if (dir_bytes > 0 && !archive.file_.ReadAt(dir.data(), dir_bytes, section_offset + 4)) {
    return bad("truncated CIFP section directory");
  }

  ByteReader dr(dir.data(), dir.size());
  archive.index_.reserve(airport_count);
  for (uint32_t i = 0; i < airport_count; ++i) {
    const uint32_t icao_off = dr.U32();
    const uint32_t icao_len = dr.U32();
    const uint64_t seg_rel = dr.U64();
    const uint32_t seg_len = dr.U32();
    if (!dr.ok()) {
      return bad("corrupt CIFP section directory");
    }
    // ICAO resolves against the global pool.
    if (static_cast<size_t>(icao_off) + icao_len > archive.pool_.size()) {
      return bad("corrupt CIFP section: ICAO reference out of range");
    }
    // The segment must lie within the CIFP section. seg_rel is relative to the
    // section start; convert to an absolute file offset. Validate the segment
    // fits in the section (offset is u64, len u32; the sum cannot overflow on a
    // 64-bit target).
    if (seg_rel > section_length || seg_len > section_length - seg_rel) {
      return bad("corrupt CIFP section: segment reference out of range");
    }
    const uint64_t abs_off = section_offset + seg_rel;
    std::string icao = archive.pool_.substr(icao_off, icao_len);
    archive.index_.emplace(std::move(icao), std::make_pair(abs_off, seg_len));
  }
  return Result<CifpArchive>::Ok(std::move(archive));
}

std::unordered_map<std::string, CifpData> CifpArchive::FetchAll() const {
  std::unordered_map<std::string, CifpData> out;
  out.reserve(index_.size());
  for (const auto& entry : index_) {
    const uint64_t offset = entry.second.first;
    const uint32_t length = entry.second.second;
    std::string bytes(length, '\0');
    if (length > 0 && !file_.ReadAt(bytes.data(), length, offset)) {
      continue;
    }
    std::optional<CifpData> data =
        DeserializeSegment(bytes.data(), bytes.size(), pool_.data(), pool_.size());
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
  // Positional read on the shared handle: pread/ReadFile take an explicit offset
  // and touch no shared cursor, so concurrent fetches for different airports are
  // race-free without a lock (contract B). Bounds were validated at OpenSection.
  std::string bytes(length, '\0');
  if (length > 0 && !file_.ReadAt(bytes.data(), length, offset)) {
    return std::nullopt;
  }
  return DeserializeSegment(bytes.data(), bytes.size(), pool_.data(), pool_.size());
}

}  // namespace bf
