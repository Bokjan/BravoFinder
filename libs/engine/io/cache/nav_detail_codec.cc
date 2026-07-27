// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/nav_detail_codec.h"

#include <algorithm>
#include <cassert>
#include <cstdint>

#include "io/cache/byte_io.h"

namespace bf {

// --- Section body layout (all string refs point into the global pool) ---------
//
//   navaid_count       U32
//   hold_count         U32
//   Navaid records [navaid_count]:
//     ident_off/len U32, region_off/len U32, kind U8, elev_ft I32,
//     freq_raw I32, range_nm F64, heading F64
//   Hold records [hold_count]:
//     fix_ident_off/len U32, fix_region_off/len U32, airport_off/len U32,
//     inbound_course F64, leg_time_min F64, leg_dist_nm F64, turn_dir U8,
//     min_alt_ft I32, max_alt_ft I32, speed_limit_kt I32

namespace {

// Wire-layout declarators for the fixed-length records in this section. These
// exist ONLY so `sizeof` is the single source of truth for the corruption fuses
// in Decode: the wire format is packed with no padding, while the in-memory types
// (NavaidDetailInfo / HoldInfo) hold std::string where the wire stores (off,len)
// ref pairs. Deliberately distinct from the in-memory structs. Encode/Decode stay
// field-by-field via ByteWriter -- never instantiate, memcpy, or take a member
// address (#pragma pack(push,1) is portable across MSVC/GCC/Clang for sizeof).
#pragma pack(push, 1)
struct WireNavaid {
  uint32_t ident_io, ident_il, region_io, region_rl;
  uint8_t kind;
  int32_t elev_ft, freq_raw;
  double range_nm, heading;
};
struct WireHold {
  uint32_t fix_io, fix_il, fix_ro, fix_rl, ap_io, ap_il;
  double inbound_course, leg_time_min, leg_dist_nm;
  uint8_t turn_dir;
  int32_t min_alt_ft, max_alt_ft, speed_limit_kt;
};
#pragma pack(pop)

constexpr size_t kNavaidRecordSize = sizeof(WireNavaid);  // 41
constexpr size_t kHoldRecordSize = sizeof(WireHold);      // 61
static_assert(kNavaidRecordSize == 41, "navaid wire layout drifted");
static_assert(kHoldRecordSize == 61, "hold wire layout drifted");

}  // namespace

Result<void> NavDetailCodec::Encode(const NavDetailArchive& archive, ByteWriter& w,
                                    StringPool& pool) {
  auto ref = [&](const std::string& s) {
    const auto r = pool.Add(s);
    w.U32(r.first);
    w.U32(r.second);
  };

  w.U32(static_cast<uint32_t>(archive.navaids_.size()));
  w.U32(static_cast<uint32_t>(archive.holds_.size()));

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
  return Result<void>::Ok();
}

Result<NavDetailArchive> NavDetailCodec::Decode(std::span<const uint8_t> body,
                                                std::span<const uint8_t> pool) {
  auto bad = [](const char* why) {
    return Result<NavDetailArchive>::Err(
        Error(ErrorCode::kCacheCorrupt, std::string(why) + "; run bf build to regenerate"));
  };

  ByteReader r(body);
  const uint32_t navaid_count = r.U32();
  const uint32_t hold_count = r.U32();
  if (!r.ok()) {
    return bad("truncated nav detail section header");
  }

  bool refs_ok = true;
  auto resolve = [&](uint32_t off, uint32_t len) -> std::string {
    return ResolveRef(pool, off, len, refs_ok);
  };

  // Sanity-check counts before allocating. Per-record sizes come from the packed
  // WireRecord declarators above (kNavaidRecordSize / kHoldRecordSize), pinned by
  // static_assert so a field add fails to compile rather than leaving a stale
  // literal here.
  if (static_cast<size_t>(navaid_count) > r.remaining() / kNavaidRecordSize) {
    return bad("corrupt nav detail section: navaid count exceeds section");
  }

  NavDetailArchive archive;

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
      return bad("truncated nav detail section navaid records");
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
  if (static_cast<size_t>(hold_count) > r.remaining() / kHoldRecordSize) {
    return bad("corrupt nav detail section: hold count exceeds section");
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
      return bad("truncated nav detail section hold records");
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
    return bad("corrupt nav detail section: string reference out of range");
  }
  // A well-formed section is consumed exactly; leftover bytes mean a count was
  // under-read (a corrupt/half-written same-version file), so reject it (mirrors
  // the graph section's trailing-byte guard).
  if (r.remaining() != 0) {
    return bad("corrupt nav detail section: trailing bytes");
  }

  archive.Finalize();
  return Result<NavDetailArchive>::Ok(std::move(archive));
}

// --- NavDetailArchive construction --------------------------------------

NavDetailArchive NavDetailArchive::FromData(const NavData& data) {
  NavDetailArchive archive;

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
    if (a.first.ident != b.first.ident) {
      return a.first.ident < b.first.ident;
    }
    return a.first.region < b.first.region;
  });
  assert(std::is_sorted(navaids_.begin(), navaids_.end(), [](const auto& a, const auto& b) {
    if (a.first.ident != b.first.ident) {
      return a.first.ident < b.first.ident;
    }
    return a.first.region < b.first.region;
  }));
  std::sort(holds_.begin(), holds_.end(),
            [](const HoldInfo& a, const HoldInfo& b) { return a.fix_ident < b.fix_ident; });
  assert(std::is_sorted(holds_.begin(), holds_.end(), [](const HoldInfo& a, const HoldInfo& b) {
    return a.fix_ident < b.fix_ident;
  }));
  loaded_ = true;
}

// --- NavDetailArchive lookup methods ------------------------------------

std::vector<NavaidDetailInfo> NavDetailArchive::FindNavaids(const std::string& ident) const {
  std::vector<NavaidDetailInfo> out;
  if (!loaded_) {
    return out;
  }
  assert(std::is_sorted(navaids_.begin(), navaids_.end(), [](const auto& a, const auto& b) {
    if (a.first.ident != b.first.ident) {
      return a.first.ident < b.first.ident;
    }
    return a.first.region < b.first.region;
  }));
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
  assert(std::is_sorted(holds_.begin(), holds_.end(), [](const HoldInfo& a, const HoldInfo& b) {
    return a.fix_ident < b.fix_ident;
  }));
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
