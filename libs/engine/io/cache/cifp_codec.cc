// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/cifp_codec.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <format>
#include <fstream>
#include <string_view>
#include <utility>
#include <vector>

#include "io/cache/byte_io.h"
#include "io/cache/cache_messages.h"
#include "io/cache/crc32c.h"

namespace bf {

namespace {

// Each of these enums is serialized as a single U8 in a segment; guard that the
// last enumerator still fits, so extending an enum past 255 fails to compile
// rather than silently truncating on write.
static_assert(static_cast<int>(ProcedureType::kApproach) < 256, "ProcedureType exceeds U8");
static_assert(static_cast<int>(PathTerminator::kUnknown) < 256, "PathTerminator exceeds U8");
static_assert(static_cast<int>(AltConstraintKind::kBetween) < 256, "AltConstraintKind exceeds U8");

// Wire-layout declarators for the fixed-length records in a segment and the
// section directory. These exist ONLY so `sizeof` is the single source of truth
// for the corruption fuses in DeserializeSegment / OpenSection: the wire format
// is packed with no padding, while the in-memory types (Procedure / ProcedureLeg
// / Runway) hold std::string and FixedIdent where the wire stores (off,len) ref
// pairs. Deliberately distinct from the in-memory structs. Encode/Decode stay
// field-by-field via ByteWriter -- never instantiate, memcpy, or take a member
// address (#pragma pack(push,1) is portable across MSVC/GCC/Clang for sizeof).
#pragma pack(push, 1)
struct WireProcedureHeader {
  uint8_t type;
  int32_t route_type;
  uint32_t name_io, name_il, trans_io, trans_il, rwy_io, rwy_il;
  uint32_t leg_count;
};
struct WireProcedureLeg {
  uint32_t fix_io, fix_il, fix_ro, fix_rl;
  uint8_t path_term;
  double course_deg, distance_nm;
  uint8_t alt_kind;
  int32_t alt1_ft, alt2_ft;
  uint16_t rnp_centinm;
  uint8_t turn_dir;
  uint16_t speed_limit_kt;
  uint8_t is_mapt;  // 0/1; format_version >= 17
};
struct WireRunway {
  uint32_t ident_io, ident_il;
  double lat, lon;
  int32_t elevation_ft;
};
struct WireDirEntry {
  uint32_t icao_off, icao_len;
  uint64_t seg_offset;
  uint32_t seg_len;
  uint32_t seg_crc;  // CRC-32C of this segment's body
};
#pragma pack(pop)

// kProcedureHeaderSize is the fixed prefix of a variable record (followed by
// leg_count * kProcedureLegSize bytes); the leg tail is fused per-procedure in
// the read loop below against the then-remaining bytes.
constexpr size_t kProcedureHeaderSize = sizeof(WireProcedureHeader);  // 33
constexpr size_t kProcedureLegSize = sizeof(WireProcedureLeg);        // 48
constexpr size_t kRunwaySize = sizeof(WireRunway);                    // 28
constexpr size_t kDirEntrySize = sizeof(WireDirEntry);                // 24
static_assert(kProcedureHeaderSize == 33, "procedure-header wire layout drifted");
static_assert(kProcedureLegSize == 48, "procedure-leg wire layout drifted");
static_assert(kRunwaySize == 28, "runway wire layout drifted");
static_assert(kDirEntrySize == 24, "directory-entry wire layout drifted");

// --- One airport's CIFP data, serialized as a bare segment body. ---
//
// Unlike the old standalone CIFP cache, a segment has NO per-segment string
// pool: every string reference points into the container's single global pool,
// so the same fix/ICAO stored across many airports is deduplicated once. A
// segment therefore cannot be deserialized in isolation -- it needs the global
// pool blob, which the CifpArchive holds in memory.

std::vector<uint8_t> SerializeSegment(const CifpData& data, StringPool& pool) {
  std::vector<uint8_t> body;
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
      ref(std::string(leg.fix.Arinc424IcaoCodeView()));
      w.U8(static_cast<uint8_t>(leg.path_term));
      w.F64(leg.course_deg);
      w.F64(leg.distance_nm);
      w.U8(static_cast<uint8_t>(leg.alt_kind));
      w.I32(leg.alt1_ft);
      w.I32(leg.alt2_ft);
      w.U16(leg.rnp_centinm);
      w.U8(static_cast<uint8_t>(leg.turn_dir));
      w.U16(leg.speed_limit_kt);
      w.U8(leg.is_mapt ? 1 : 0);
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
// global pool blob (`pool`).
Result<CifpData> DeserializeSegment(std::span<const uint8_t> data, std::span<const uint8_t> pool) {
  ByteReader br(data);
  auto bad = [](std::string_view message) {
    return Result<CifpData>::Err(Error(ErrorCode::kCacheCorrupt, std::string(message)));
  };

  bool refs_ok = true;
  auto ref = [&](std::string& s) {
    const uint32_t off = br.U32();
    const uint32_t len = br.U32();
    s = ResolveRef(pool, off, len, refs_ok);
  };
  // Same read, returning the resolved string by value -- used for the fix, which
  // is a FixedIdent (cannot bind to std::string&) built via FromParts.
  auto read_ref = [&]() -> std::string {
    const uint32_t off = br.U32();
    const uint32_t len = br.U32();
    return ResolveRef(pool, off, len, refs_ok);
  };

  // Minimum on-disk bytes per record, used to reject an absurd count before
  // resizing (ByteReader still guards the actual reads, but this stops a forged
  // count from forcing a huge allocation). Sizes come from the packed WireRecord
  // declarators above (kProcedureHeaderSize / kProcedureLegSize / kRunwaySize),
  // pinned by static_assert so a field add fails to compile rather than leaving
  // a stale literal here. The procedure header is a fixed prefix of a variable
  // record; its leg tail is fused per-procedure in the loop below.
  auto count_fits = [&](uint32_t count, size_t per_record) {
    return static_cast<size_t>(count) <= br.remaining() / per_record;
  };

  CifpData data_out;
  const uint32_t proc_count = br.U32();
  if (!br.ok() || !count_fits(proc_count, kProcedureHeaderSize)) {
    return bad("invalid procedure count");
  }
  data_out.procedures.resize(proc_count);
  for (uint32_t i = 0; i < proc_count; ++i) {
    Procedure& p = data_out.procedures[i];
    const uint8_t type_byte = br.U8();
    // Reject an out-of-range enum byte instead of reinterpreting it; a corrupt
    // or half-written same-version file could otherwise inject an invalid
    // ProcedureType that downstream switches handle as a silent wrong branch.
    if (type_byte > static_cast<uint8_t>(ProcedureType::kApproach)) {
      return bad("invalid procedure type");
    }
    p.type = static_cast<ProcedureType>(type_byte);
    p.route_type = br.I32();
    ref(p.name);
    ref(p.transition_ident);
    ref(p.runway);
    const uint32_t leg_count = br.U32();
    if (!br.ok() || !count_fits(leg_count, kProcedureLegSize)) {
      return bad("invalid procedure leg count");
    }
    p.legs.resize(leg_count);
    for (uint32_t j = 0; j < leg_count; ++j) {
      ProcedureLeg& leg = p.legs[j];
      // Two pool refs (ident, region) -> FixedIdent, order matching Encode.
      const std::string fix_ident = read_ref();
      const std::string fix_arinc424_icao_code = read_ref();
      Result<FixedIdent> fixed = FixedIdent::FromParts(fix_ident, fix_arinc424_icao_code);
      if (!fixed) {
        return Result<CifpData>::Err(std::move(fixed).error());
      }
      leg.fix = std::move(fixed).value();
      const uint8_t path_byte = br.U8();
      if (path_byte > static_cast<uint8_t>(PathTerminator::kUnknown)) {
        return bad("invalid procedure path terminator");
      }
      leg.path_term = static_cast<PathTerminator>(path_byte);
      leg.course_deg = static_cast<float>(br.F64());
      leg.distance_nm = static_cast<float>(br.F64());
      const uint8_t alt_byte = br.U8();
      if (alt_byte > static_cast<uint8_t>(AltConstraintKind::kBetween)) {
        return bad("invalid procedure altitude constraint");
      }
      leg.alt_kind = static_cast<AltConstraintKind>(alt_byte);
      leg.alt1_ft = br.I32();
      leg.alt2_ft = br.I32();
      leg.rnp_centinm = br.U16();
      leg.turn_dir = static_cast<char>(br.U8());
      leg.speed_limit_kt = br.U16();
      // Wire is_mapt is 0/1 only; any other byte is corrupt (not "truthy").
      const uint8_t mapt = br.U8();
      if (mapt > 1) {
        return bad("invalid procedure MAPT flag");
      }
      leg.is_mapt = mapt != 0;
    }
  }
  const uint32_t rwy_count = br.U32();
  if (!br.ok() || !count_fits(rwy_count, kRunwaySize)) {
    return bad("invalid runway count");
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
    return bad("invalid field or string reference");
  }
  // A well-formed segment body is consumed exactly; leftover bytes mean a count
  // was under-read (a corrupt/half-written same-version file), so reject it
  // (mirrors the graph / nav-detail section trailing-byte guards).
  if (br.remaining() != 0) {
    return bad("trailing bytes");
  }
  return Result<CifpData>::Ok(std::move(data_out));
}

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
    std::vector<uint8_t> body;
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

  std::vector<uint8_t> section;
  ByteWriter sw(section);
  sw.U32(static_cast<uint32_t>(entries.size()));
  uint64_t running = segments_start;
  for (size_t i = 0; i < entries.size(); ++i) {
    sw.U32(icao_refs[i].first);
    sw.U32(icao_refs[i].second);
    sw.U64(running);  // relative to CIFP section start
    sw.U32(static_cast<uint32_t>(entries[i].body.size()));
    sw.U32(Crc32C::Compute(entries[i].body));  // segment body integrity
    running += entries[i].body.size();
  }
  for (const Entry& e : entries) {
    section.insert(section.end(), e.body.begin(), e.body.end());
  }

  w.Bytes(section.data(), section.size());
  return Result<uint32_t>::Ok(static_cast<uint32_t>(entries.size()));
}

Result<CifpArchive> CifpCodec::OpenSection(const std::string& path, uint64_t section_offset,
                                           uint64_t section_length, uint32_t section_crc,
                                           std::vector<uint8_t> pool_blob) {
  auto bad = [&](std::string_view why) {
    return Result<CifpArchive>::Err(Error(ErrorCode::kCacheCorrupt, WithRebuildHint(why)));
  };

  // Open a positional-read handle on the unified file and read the CIFP section's
  // directory. The per-airport segments stay on disk and are fetched lazily.
  CifpArchive archive;
  archive.file_ = PreadFile(path);
  if (!archive.file_.is_open()) {
    return Result<CifpArchive>::Err(
        Error(ErrorCode::kDataMissing,
              std::format("cannot open .bfdb for CIFP segment reads: {}", path)));
  }
  archive.pool_ = std::move(pool_blob);

  if (section_length < 4) {
    return bad("corrupt CIFP section: too short");
  }
  // Read airport_count.
  uint8_t count_buf[4];
  if (!archive.file_.ReadAt(count_buf, section_offset)) {
    return bad("truncated CIFP section directory");
  }
  ByteReader cr(count_buf);
  const uint32_t airport_count = cr.U32();
  // A directory row is kDirEntrySize bytes; reject a count that could not fit in
  // the section before allocating.
  if (static_cast<uint64_t>(airport_count) > (section_length - 4) / kDirEntrySize) {
    return bad("corrupt CIFP section: directory count exceeds section size");
  }

  // Read the whole directory region in one positional read.
  const size_t dir_bytes = static_cast<size_t>(airport_count) * kDirEntrySize;
  std::vector<uint8_t> dir(dir_bytes);
  if (dir_bytes > 0 && !archive.file_.ReadAt(dir, section_offset + 4)) {
    return bad("truncated CIFP section directory");
  }

  // Verify the directory prefix (airport_count + directory rows) against the
  // section-table CRC. Segments are lazy and carry their own per-segment CRC, so
  // this CRC covers only what Open reads now; a section-level CRC over the whole
  // body would force reading every segment here, defeating lazy fetch. Incremental
  // update folds count_buf and dir without copying them into one buffer.
  uint32_t dir_crc = Crc32C::Update(0, count_buf);
  dir_crc = Crc32C::Update(dir_crc, dir);
  if (dir_crc != section_crc) {
    return bad("corrupt CIFP section: directory CRC mismatch");
  }

  ByteReader dr(dir);
  archive.index_.reserve(airport_count);
  for (uint32_t i = 0; i < airport_count; ++i) {
    const uint32_t icao_off = dr.U32();
    const uint32_t icao_len = dr.U32();
    const uint64_t seg_rel = dr.U64();
    const uint32_t seg_len = dr.U32();
    const uint32_t seg_crc = dr.U32();
    if (!dr.ok()) {
      return bad("corrupt CIFP section directory");
    }
    // ICAO resolves against the global pool. Use the subtraction form (as
    // ResolveRef does) so a near-UINT32_MAX icao_off cannot wrap the sum past
    // pool_.size() on a 32-bit size_t and slip the bounds check.
    if (icao_off > archive.pool_.size() || icao_len > archive.pool_.size() - icao_off) {
      return bad("corrupt CIFP section: ICAO reference out of range");
    }
    // The segment must lie within the CIFP section. seg_rel is relative to the
    // section start; convert to an absolute file offset. Validate the segment
    // fits in the section (offset is u64, len u32; the sum cannot overflow on a
    // 64-bit target).
    if (seg_rel > section_length || seg_len > section_length - seg_rel) {
      return bad("corrupt CIFP section: segment reference out of range");
    }
    // A non-empty segment must also start at or after the count + directory
    // region; a seg_rel pointing into the directory would alias directory bytes
    // as a segment body. Encode never emits this -- it only guards a forged or
    // corrupt same-version file (the upper-bound check above misses it).
    const uint64_t body_start = 4 + static_cast<uint64_t>(airport_count) * kDirEntrySize;
    if (seg_len > 0 && seg_rel < body_start) {
      return bad("corrupt CIFP section: segment overlaps directory");
    }
    const uint64_t abs_off = section_offset + seg_rel;
    // ICAO bounds were validated above, so the slice is in range. An over-cap
    // pool string cannot form a valid fixed-width lookup key.
    const std::string_view icao_sv(reinterpret_cast<const char*>(archive.pool_.data() + icao_off),
                                   icao_len);
    Result<FixedName8> key = FixedName8::From(icao_sv);
    if (!key) {
      return bad(std::format("corrupt CIFP section: invalid ICAO key: {}", key.error().message));
    }
    archive.index_.emplace_back(std::move(key).value(),
                                CifpArchive::SegmentLoc{abs_off, seg_len, seg_crc});
  }
  std::sort(archive.index_.begin(), archive.index_.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
  return Result<CifpArchive>::Ok(std::move(archive));
}

size_t CifpCodec::DirectoryPrefixLen(uint32_t airport_count) {
  return 4 + static_cast<size_t>(airport_count) * kDirEntrySize;
}

Result<std::unordered_map<std::string, CifpData>> CifpArchive::FetchAll() const {
  std::unordered_map<std::string, CifpData> out;
  out.reserve(index_.size());
  for (const auto& entry : index_) {
    const SegmentLoc& loc = entry.second;
    std::vector<uint8_t> bytes(loc.len);
    if (loc.len > 0 && !file_.ReadAt(bytes, loc.abs_off)) {
      return Result<std::unordered_map<std::string, CifpData>>::Err(Error(
          ErrorCode::kCacheCorrupt,
          WithRebuildHint(std::format("CIFP segment read failed for {}", entry.first.View()))));
    }
    if (Crc32C::Compute(bytes) != loc.crc) {
      return Result<std::unordered_map<std::string, CifpData>>::Err(Error(
          ErrorCode::kCacheCorrupt,
          WithRebuildHint(std::format("CIFP segment CRC mismatch for {}", entry.first.View()))));
    }
    Result<CifpData> data = DeserializeSegment(bytes, pool_);
    if (!data) {
      return Result<std::unordered_map<std::string, CifpData>>::Err(
          Error(ErrorCode::kCacheCorrupt,
                WithRebuildHint(std::format("CIFP segment decode failed for {}: {}",
                                            entry.first.View(), data.error().message))));
    }
    out.emplace(std::string(entry.first.View()), std::move(data).value());
  }
  return Result<std::unordered_map<std::string, CifpData>>::Ok(std::move(out));
}

const CifpArchive::SegmentLoc* CifpArchive::Find(const std::string& icao) const {
  Result<FixedName8> key_result = FixedName8::From(icao);
  if (!key_result) {
    return nullptr;
  }
  const FixedName8& key = key_result.value();
  auto it = std::lower_bound(
      index_.begin(), index_.end(), key,
      [](const std::pair<FixedName8, SegmentLoc>& e, const FixedName8& k) { return e.first < k; });
  if (it != index_.end() && it->first == key) {
    return &it->second;
  }
  return nullptr;
}

Result<std::optional<CifpData>> CifpArchive::Fetch(const std::string& icao) const {
  const SegmentLoc* loc = Find(icao);
  if (loc == nullptr) {
    return Result<std::optional<CifpData>>::Ok(std::nullopt);
  }
  // Positional read on the shared handle: pread/ReadFile take an explicit offset
  // and touch no shared cursor, so concurrent fetches for different airports are
  // race-free without a lock (thread-safety contract). Bounds were validated at OpenSection.
  std::vector<uint8_t> bytes(loc->len);
  if (loc->len > 0 && !file_.ReadAt(bytes, loc->abs_off)) {
    return Result<std::optional<CifpData>>::Err(
        Error(ErrorCode::kCacheCorrupt,
              WithRebuildHint(std::format("CIFP segment read failed for {}", icao))));
  }
  // Verify the segment body before deserializing -- a bit flip into another valid
  // value silently corrupts a procedure/leg field that the decode fuses accept.
  if (Crc32C::Compute(bytes) != loc->crc) {
    return Result<std::optional<CifpData>>::Err(
        Error(ErrorCode::kCacheCorrupt,
              WithRebuildHint(std::format("CIFP segment CRC mismatch for {}", icao))));
  }
  Result<CifpData> data = DeserializeSegment(bytes, pool_);
  if (!data) {
    return Result<std::optional<CifpData>>::Err(
        Error(ErrorCode::kCacheCorrupt,
              WithRebuildHint(std::format("CIFP segment decode failed for {}: {}", icao,
                                          data.error().message))));
  }
  return Result<std::optional<CifpData>>::Ok(std::move(data).value());
}

}  // namespace bf
