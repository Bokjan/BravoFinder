#include "io/cache/nav_detail_cache.h"

#include <algorithm>
#include <cstring>
#include <fstream>

#include "io/cache/byte_io.h"

namespace bf {

namespace {

constexpr char kMagic[4] = {'B', 'F', 'N', 'D'};

// WaypointKind is serialized as a single U8 per navaid record; guard it fits.
static_assert(static_cast<int>(WaypointKind::kOther) < 256, "WaypointKind exceeds U8");

}  // namespace

// --- Serialization format (version 1) -----------------------------------
//
// Header:
//   magic[4]           "BFND"
//   format_version     U32
//   cycle              U32
//   build              U32
//   navaid_count       U32
//   hold_count         U32
//   program_semver     U32-len + bytes
//   source_loader      U32-len + bytes
//   string_pool_size   U32
//
// Navaid records [navaid_count]:
//   ident_offset U32, ident_len U32
//   region_offset U32, region_len U32
//   kind         U8  (WaypointKind)
//   elev_ft      I32
//   freq_raw     I32
//   range_nm     F64
//   heading      F64
//
// Hold records [hold_count]:
//   fix_ident_offset U32, fix_ident_len U32
//   fix_region_offset U32, fix_region_len U32
//   airport_offset U32, airport_len U32
//   inbound_course F64
//   leg_time_min   F64
//   leg_dist_nm    F64
//   turn_dir       U8  (0='R', 1='L')
//   min_alt_ft     I32
//   max_alt_ft     I32
//   speed_limit_kt I32
//
// String pool blob (all strings concatenated without separators)

Result<void> NavDetailCache::Build(const std::string& out_path, const NavDetailArchive& archive,
                                   const std::string& source_loader,
                                   const std::string& program_semver) {
  StringPool pool;
  std::string body;
  ByteWriter w(body);
  // Navaid record 41 B + hold record 65 B on disk (excluding pool bytes); a hint
  // to avoid repeated reallocation.
  w.Reserve(archive.navaids_.size() * 41 + archive.holds_.size() * 65);

  auto ref = [&](const std::string& s) {
    const auto r = pool.Add(s);
    w.U32(r.first);
    w.U32(r.second);
  };

  // Navaid records (from the archive's sorted navaids_; the pair key mirrors
  // the Info's ident/region, so serialize the Info directly).
  for (const auto& entry : archive.navaids_) {
    const NavaidDetailInfo& d = entry.second;
    ref(d.ident);
    ref(d.region);
    w.U8(static_cast<uint8_t>(d.kind));
    w.I32(d.elev_ft);
    w.I32(d.freq_raw);
    w.F64(d.range_nm);
    w.F64(d.heading);
  }

  // Hold records
  for (const HoldInfo& h : archive.holds_) {
    ref(h.fix_ident);
    ref(h.fix_region);
    ref(h.airport_icao);
    w.F64(h.inbound_course);
    w.F64(h.leg_time_min);
    w.F64(h.leg_dist_nm);
    w.U8(h.turn_dir == 'L' ? 1 : 0);
    w.I32(h.min_alt_ft);
    w.I32(h.max_alt_ft);
    w.I32(h.speed_limit_kt);
  }

  // Assemble file: header + body + pool
  std::string out;
  ByteWriter hw(out);
  out.append(kMagic, 4);
  hw.U32(kFormatVersion);
  hw.U32(archive.cycle_);
  hw.U32(archive.build_);
  hw.U32(static_cast<uint32_t>(archive.navaids_.size()));
  hw.U32(static_cast<uint32_t>(archive.holds_.size()));

  hw.Str(program_semver);
  hw.Str(source_loader);

  hw.U32(static_cast<uint32_t>(pool.blob().size()));
  out.append(body);
  out.append(pool.blob());

  std::ofstream f(out_path, std::ios::binary | std::ios::trunc);
  if (!f.is_open()) {
    return Result<void>::Err(
        Error(ErrorCode::kDataMissing, "cannot open nav detail cache for writing: " + out_path));
  }
  f.write(out.data(), static_cast<std::streamsize>(out.size()));
  if (!f) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "failed writing " + out_path));
  }
  return Result<void>::Ok();
}

Result<NavDetailArchive> NavDetailCache::Open(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    return Result<NavDetailArchive>::Err(
        Error(ErrorCode::kDataMissing, "cannot open nav detail cache: " + path));
  }
  const std::streamoff file_size = f.tellg();
  f.seekg(0);

  auto bad = [&](const char* why) {
    return Result<NavDetailArchive>::Err(
        Error(ErrorCode::kDataMissing, std::string(why) + "; run bf build to regenerate"));
  };

  // Read entire file into memory for a single-pass parse.
  if (file_size <= 0) {
    return bad("empty nav detail cache");
  }
  std::string buf(static_cast<size_t>(file_size), '\0');
  f.read(buf.data(), file_size);
  if (!f) {
    return bad("truncated nav detail cache");
  }

  ByteReader r(buf.data(), buf.size());

  // Magic
  if (r.remaining() < 4) {
    return bad("not a nav detail cache (too short)");
  }
  char magic[4];
  for (int i = 0; i < 4; ++i) {
    magic[i] = static_cast<char>(r.U8());
  }
  if (std::memcmp(magic, kMagic, 4) != 0) {
    return bad("not a nav detail cache (bad magic)");
  }

  const uint32_t format = r.U32();
  if (!r.ok()) {
    return bad("truncated nav detail cache header");
  }
  if (format != kFormatVersion) {
    return bad("incompatible nav detail cache format version");
  }

  NavDetailArchive archive;
  archive.cycle_ = r.U32();
  archive.build_ = r.U32();
  const uint32_t navaid_count = r.U32();
  const uint32_t hold_count = r.U32();
  if (!r.ok()) {
    return bad("truncated nav detail cache header");
  }

  // Inline strings: program_semver and source_loader (lengths then bytes).
  // Read and discarded here -- the archive does not retain provenance.
  const std::string program_semver_unused = r.Str();
  const std::string source_loader_unused = r.Str();
  (void)program_semver_unused;
  (void)source_loader_unused;
  if (!r.ok()) {
    return bad("corrupt nav detail cache header");
  }

  const uint32_t pool_size = r.U32();
  if (!r.ok() || pool_size > r.remaining()) {
    return bad("corrupt nav detail cache: pool size exceeds file");
  }

  // Sanity check counts before allocating
  constexpr size_t kNavaidRecordMin = 8 + 8 + 1 + 4 + 4 + 8 + 8;            // 41 bytes
  constexpr size_t kHoldRecordMin = 8 + 8 + 8 + 8 + 8 + 8 + 1 + 4 + 4 + 4;  // 65 bytes
  if (static_cast<size_t>(navaid_count) > r.remaining() / kNavaidRecordMin) {
    return bad("corrupt nav detail cache: navaid count exceeds file");
  }

  // Records section comes before the pool blob in the buffer.
  // Resolve string references against the pool blob at the end of the file,
  // in place (a slice of the already-read buffer), no separate copy.
  const size_t pool_start = static_cast<size_t>(file_size) - pool_size;
  const char* pool_blob = buf.data() + pool_start;

  bool refs_ok = true;
  auto resolve = [&](uint32_t off, uint32_t len) -> std::string {
    return ResolveRef(pool_blob, pool_size, off, len, refs_ok);
  };

  // Navaid records
  archive.navaids_.resize(navaid_count);
  for (uint32_t i = 0; i < navaid_count; ++i) {
    const uint32_t ident_off = r.U32();
    const uint32_t ident_len = r.U32();
    const uint32_t region_off = r.U32();
    const uint32_t region_len = r.U32();
    const auto kind = static_cast<WaypointKind>(r.U8());
    const int32_t elev_ft = r.I32();
    const int32_t freq_raw = r.I32();
    const double range_nm = r.F64();
    const double heading = r.F64();
    if (!r.ok()) {
      return bad("truncated nav detail cache navaid records");
    }
    NavaidDetailInfo info;
    info.ident = resolve(ident_off, ident_len);
    info.region = resolve(region_off, region_len);
    info.kind = kind;
    info.elev_ft = elev_ft;
    info.freq_raw = freq_raw;
    info.range_nm = range_nm;
    info.heading = heading;
    archive.navaids_[i] = {Ident(info.ident, info.region), std::move(info)};
  }

  // Hold records
  if (static_cast<size_t>(hold_count) > r.remaining() / kHoldRecordMin) {
    return bad("corrupt nav detail cache: hold count exceeds file");
  }
  archive.holds_.resize(hold_count);
  for (uint32_t i = 0; i < hold_count; ++i) {
    const uint32_t fix_ident_off = r.U32();
    const uint32_t fix_ident_len = r.U32();
    const uint32_t fix_region_off = r.U32();
    const uint32_t fix_region_len = r.U32();
    const uint32_t airport_off = r.U32();
    const uint32_t airport_len = r.U32();
    const double inbound_course = r.F64();
    const double leg_time_min = r.F64();
    const double leg_dist_nm = r.F64();
    const uint8_t turn_raw = r.U8();
    const int32_t min_alt_ft = r.I32();
    const int32_t max_alt_ft = r.I32();
    const int32_t speed_limit_kt = r.I32();
    if (!r.ok()) {
      return bad("truncated nav detail cache hold records");
    }
    HoldInfo& h = archive.holds_[i];
    h.fix_ident = resolve(fix_ident_off, fix_ident_len);
    h.fix_region = resolve(fix_region_off, fix_region_len);
    h.airport_icao = resolve(airport_off, airport_len);
    h.inbound_course = inbound_course;
    h.leg_time_min = leg_time_min;
    h.leg_dist_nm = leg_dist_nm;
    h.turn_dir = (turn_raw == 1) ? 'L' : 'R';
    h.min_alt_ft = min_alt_ft;
    h.max_alt_ft = max_alt_ft;
    h.speed_limit_kt = speed_limit_kt;
  }

  if (!refs_ok) {
    return bad("corrupt nav detail cache: string reference out of range");
  }

  archive.Finalize();
  return Result<NavDetailArchive>::Ok(std::move(archive));
}

// --- NavDetailArchive construction --------------------------------------

NavDetailArchive NavDetailArchive::FromData(const NavData& data) {
  NavDetailArchive archive;
  archive.cycle_ = data.cycle;
  archive.build_ = data.build;

  archive.navaids_.reserve(data.navaid_details.size());
  for (const NavaidDetail& d : data.navaid_details) {
    NavaidDetailInfo info;
    info.ident = d.ident.ident;
    info.region = d.ident.region;
    info.kind = d.kind;
    info.elev_ft = d.elev_ft;
    info.freq_raw = d.freq_raw;
    info.range_nm = d.range_nm;
    info.heading = d.heading;
    archive.navaids_.emplace_back(d.ident, std::move(info));
  }

  archive.holds_.reserve(data.hold_fixes.size());
  for (const HoldFix& h : data.hold_fixes) {
    HoldInfo info;
    info.fix_ident = h.fix.ident;
    info.fix_region = h.fix.region;
    info.airport_icao = h.airport_icao;
    info.inbound_course = h.inbound_course;
    info.leg_time_min = h.leg_time_min;
    info.leg_dist_nm = h.leg_dist_nm;
    info.turn_dir = h.turn_dir;
    info.min_alt_ft = h.min_alt_ft;
    info.max_alt_ft = h.max_alt_ft;
    info.speed_limit_kt = h.speed_limit_kt;
    archive.holds_.push_back(std::move(info));
  }

  archive.Finalize();
  return archive;
}

void NavDetailArchive::Finalize() {
  // Sort both arrays for binary-search lookup.
  std::sort(navaids_.begin(), navaids_.end(), [](const auto& a, const auto& b) {
    if (a.first.ident != b.first.ident) return a.first.ident < b.first.ident;
    return a.first.region < b.first.region;
  });
  std::sort(holds_.begin(), holds_.end(),
            [](const HoldInfo& a, const HoldInfo& b) { return a.fix_ident < b.fix_ident; });
  loaded_ = true;
}

// --- NavDetailArchive lookup methods ------------------------------------

std::vector<NavaidDetailInfo> NavDetailArchive::FindNavaids(const std::string& ident) const {
  std::vector<NavaidDetailInfo> out;
  if (!loaded_) {
    return out;
  }
  // lower_bound on ident string; collect all matching ident (any region).
  auto it = std::lower_bound(
      navaids_.begin(), navaids_.end(), ident,
      [](const auto& entry, const std::string& key) { return entry.first.ident < key; });
  for (; it != navaids_.end() && it->first.ident == ident; ++it) {
    out.push_back(it->second);
  }
  return out;
}

std::vector<HoldInfo> NavDetailArchive::FindHolds(const std::string& ident) const {
  std::vector<HoldInfo> out;
  if (!loaded_) {
    return out;
  }
  // holds_ is sorted by fix_ident; use lower/upper_bound to find the range.
  auto cmp = [](const HoldInfo& h, const std::string& key) { return h.fix_ident < key; };
  auto lo = std::lower_bound(holds_.begin(), holds_.end(), ident, cmp);
  auto hi =
      std::upper_bound(lo, holds_.end(), ident,
                       [](const std::string& key, const HoldInfo& h) { return key < h.fix_ident; });
  out.assign(lo, hi);
  return out;
}

}  // namespace bf
