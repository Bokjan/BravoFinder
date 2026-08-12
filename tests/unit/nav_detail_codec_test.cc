// SPDX-License-Identifier: MIT
#include "io/cache/nav_detail_codec.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string>
#include <vector>

#include "core/domain/hold_fix.h"
#include "core/domain/navaid_detail.h"
#include "io/cache/byte_io.h"
#include "io/loaders/nav_data.h"

namespace {

// Build a small NavData with a couple of navaids (one sharing an ident across
// regions) and a couple of holds (one fix carrying two holds), enough to
// exercise the sorted-array multi-value lookups.
bf::NavData MakeSampleData() {
  bf::NavData d;
  d.cycle = 2601;

  d.navaid_details.push_back(
      bf::NavaidDetail{bf::Ident("SEA", "K1"), bf::WaypointKind::kVor, 354, 11680, 150.0, 19.0});
  d.navaid_details.push_back(
      bf::NavaidDetail{bf::Ident("DGC", "K2"), bf::WaypointKind::kVor, 500, 11500, 130.0, -3.0});
  // Same ident, different region -> both must come back from a bare-ident query.
  d.navaid_details.push_back(
      bf::NavaidDetail{bf::Ident("DGC", "LF"), bf::WaypointKind::kNdb, 0, 350, 50.0, 0.0});

  bf::HoldFix h1;
  h1.fix = bf::Ident("AE701", "DA");
  h1.airport_icao = "DAAE";
  h1.inbound_course = 171.0;
  h1.leg_time_min = 1.0;
  h1.leg_dist_nm = 0.0;
  h1.turn_dir = 'R';
  h1.min_alt_ft = 5580;
  h1.max_alt_ft = 14000;
  h1.speed_limit_kt = 230;
  d.hold_fixes.push_back(h1);

  // A second hold at the same fix ident (different parameters) to exercise the
  // multi-value equal-range lookup.
  bf::HoldFix h2 = h1;
  h2.inbound_course = 351.0;
  h2.turn_dir = 'L';
  d.hold_fixes.push_back(h2);

  // An enroute hold at a different fix.
  bf::HoldFix h3;
  h3.fix = bf::Ident("BOTON", "LF");
  h3.airport_icao = "ENRT";
  h3.inbound_course = 90.0;
  h3.leg_time_min = 0.0;
  h3.leg_dist_nm = 7.0;
  h3.turn_dir = 'R';
  h3.min_alt_ft = 3000;
  h3.max_alt_ft = 0;  // no upper limit
  h3.speed_limit_kt = 0;
  d.hold_fixes.push_back(h3);

  return d;
}

// Encode an archive to a section body + pool, then decode it back -- the unit
// under test is the section codec, with the pool played by a standalone
// StringPool as the unified container would provide.
bf::Result<bf::NavDetailArchive> RoundTrip(const bf::NavDetailArchive& src) {
  bf::StringPool pool;
  std::vector<uint8_t> body;
  bf::ByteWriter w(body);
  bf::Result<void> enc = bf::NavDetailCodec::Encode(src, w, pool);
  if (!enc) {
    return bf::Result<bf::NavDetailArchive>::Err(std::move(enc).error());
  }
  return bf::NavDetailCodec::Decode(body, pool.blob());
}

}  // namespace

TEST_CASE("nav detail codec: encode -> decode round-trips navaids", "[unit][detail]") {
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(MakeSampleData());
  bf::Result<bf::NavDetailArchive> opened = RoundTrip(src);
  REQUIRE(opened);
  const bf::NavDetailArchive& a = opened.value();

  // Single-region navaid.
  auto sea = a.FindNavaids("SEA");
  REQUIRE(sea.size() == 1);
  CHECK(sea[0].ident == "SEA");
  CHECK(sea[0].arinc424_icao_code == "K1");
  CHECK(sea[0].kind == bf::WaypointKind::kVor);
  CHECK(sea[0].elev_ft == 354);
  CHECK(sea[0].freq_raw == 11680);
  CHECK(sea[0].range_nm == 150.0);
  CHECK(sea[0].heading == 19.0);

  // Ident reused across regions returns both matches.
  auto dgc = a.FindNavaids("DGC");
  REQUIRE(dgc.size() == 2);

  // Unknown ident -> empty.
  CHECK(a.FindNavaids("NOPE").empty());
}

TEST_CASE("nav detail codec: encode -> decode round-trips holds", "[unit][detail]") {
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(MakeSampleData());
  bf::Result<bf::NavDetailArchive> opened = RoundTrip(src);
  REQUIRE(opened);
  const bf::NavDetailArchive& a = opened.value();

  // A fix ident with two holds returns both (multi-value equal-range).
  auto ae = a.FindHolds("AE701");
  REQUIRE(ae.size() == 2);
  for (const bf::HoldInfo& h : ae) {
    CHECK(h.fix_ident == "AE701");
    CHECK(h.fix_arinc424_icao_code == "DA");
    CHECK(h.airport_icao == "DAAE");
    CHECK(h.min_alt_ft == 5580);
    CHECK(h.max_alt_ft == 14000);
    CHECK(h.speed_limit_kt == 230);
  }
  // The two differ in inbound course and turn direction.
  CHECK(ae[0].turn_dir != ae[1].turn_dir);

  // An enroute hold with no upper limit / no speed limit.
  auto boton = a.FindHolds("BOTON");
  REQUIRE(boton.size() == 1);
  CHECK(boton[0].airport_icao == "ENRT");
  CHECK(boton[0].leg_dist_nm == 7.0);
  CHECK(boton[0].max_alt_ft == 0);
  CHECK(boton[0].speed_limit_kt == 0);

  CHECK(a.FindHolds("NOPE").empty());
}

TEST_CASE("nav detail codec: empty data round-trips to an empty archive", "[unit][detail]") {
  bf::NavData empty;
  empty.cycle = 2601;
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(empty);
  bf::Result<bf::NavDetailArchive> opened = RoundTrip(src);
  REQUIRE(opened);
  CHECK(opened.value().FindNavaids("SEA").empty());
  CHECK(opened.value().FindHolds("AE701").empty());
}

TEST_CASE("nav detail codec: a byte-corrupted navaid kind is rejected", "[unit][detail]") {
  // Decode must reject an out-of-range WaypointKind byte (mirrors graph_codec's
  // vertex-kind guard) instead of reinterpreting it into a lookup slot. The
  // threat is a valid encoding byte-corrupted on disk (bit-flip / half-write).
  bf::NavDetailArchive src = bf::NavDetailArchive::FromData(MakeSampleData());
  bf::StringPool pool;
  std::vector<uint8_t> body;
  bf::ByteWriter w(body);
  REQUIRE(bf::NavDetailCodec::Encode(src, w, pool));

  // Header (navaid_count U32 + hold_count U32 = 8) then the first navaid record:
  // 4 ref U32s (ident_off/ident_len/arinc424_icao_code_off/arinc424_icao_code_len = 16) -> kind U8
  // at 24.
  constexpr size_t kFirstKindOffset = 8 + 4 * 4;
  static_assert(kFirstKindOffset == 24, "nav detail wire layout drifted");
  REQUIRE(body.size() > kFirstKindOffset);
  body[kFirstKindOffset] = 0xFF;  // > WaypointKind::kOther (4)
  bf::Result<bf::NavDetailArchive> r = bf::NavDetailCodec::Decode(body, pool.blob());
  CHECK_FALSE(r);
  if (!r) {
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
  }
}
