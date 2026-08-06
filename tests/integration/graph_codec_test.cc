// SPDX-License-Identifier: MIT
#include "io/cache/graph_codec.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "core/base/env.h"
#include "core/domain/coordinate.h"
#include "core/domain/fixed_string.h"
#include "core/domain/waypoint.h"
#include "core/graph/nav_graph.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/cache/byte_io.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/unified_cache.h"
#include "io/nav_database.h"
#include "test_xplane12.h"

namespace {

using bf::test::EnsureXPlane12;
using bf::test::NavDataDir;

// A unique temp path for a .bfdb produced by a test. Uses the test name so
// parallel cases do not collide. Uses the platform temp dir so the test runs on
// Windows (where /tmp is absent).
std::string TempBfdb(const std::string& tag) {
  std::filesystem::path dir = std::filesystem::temp_directory_path();
  return (dir / ("bravofinder_test_" + tag + ".bfdb")).string();
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

// Build a database and write it to a unified .bfdb, returning the path (empty on
// SKIP). Includes the CIFP section so the cached path resolves procedures.
std::string BuildCache(const std::string& tag) {
  const std::string dir = EnsureXPlane12();
  bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(dir);
  if (!db) {
    return {};
  }
  const std::string path = TempBfdb(tag);
  bf::Result<uint32_t> written = db.value().WriteUnified(path);
  REQUIRE(written);
  return path;
}

// A minimal but structurally valid graph snapshot: two vertices, one DCT edge
// 0->1, one airway name (index 0), an empty MSA list, and a default (all-zero)
// MORA grid. Every field is in range, so Decode of its encoding must succeed;
// the corruption cases below break exactly one field each.
bf::GraphSnapshot MakeValidSnapshot() {
  bf::GraphSnapshot s;
  s.first_airport_vertex = 2;  // no airport vertices -> no elevation records
  s.coords = {bf::Coordinate{40.0, -73.0}, bf::Coordinate{34.0, -118.0}};
  s.offsets = {0, 1, 1};  // vertex 0 has edge [0,1); vertex 1 has none
  bf::GraphEdge edge;
  edge.to = 1;
  edge.distance_nm = 10.0f;
  edge.airway_id = 0;  // DCT
  edge.level = bf::AirwayLevel::kLow;
  s.edges = {edge};
  s.has_outbound = {1, 0};
  s.has_inbound = {0, 1};
  s.idents = {bf::FixedIdent::FromParts("AAAAA", "K1"), bf::FixedIdent::FromParts("BBBBB", "K2")};
  s.kinds = {bf::WaypointKind::kFix, bf::WaypointKind::kFix};
  s.airway_names = {"DCT"};
  return s;
}

// Encode a snapshot to its section body + the shared string-pool blob. REQUIREs
// the encode to succeed (callers pass a structurally and semantically valid
// snapshot; the encode-rejection cases below call Encode directly).
void EncodeSnapshot(const bf::GraphSnapshot& s, std::vector<uint8_t>* body,
                    std::vector<uint8_t>* pool_blob) {
  bf::ByteWriter w(*body);
  bf::StringPool pool;
  REQUIRE(bf::GraphCodec::Encode(s, w, pool));
  const auto blob = pool.blob();
  pool_blob->assign(blob.begin(), blob.end());
}

// A snapshot that exercises every on-disk field with non-default values: an
// airport vertex (first_airport_vertex != V), a two-way CSR edge with non-zero
// altitude band, a non-default MSA sector, and a non-zero MORA cell. The
// per-field assertions in "every field survives the round-trip" prove each
// field survives with the exact value Encode wrote -- catching an Encode/Decode
// drift that leaves sizes and byte counts unchanged (the "same-width field
// swap" family), which the Wire static_asserts, the trailing-bytes guard, and a
// self-comparing round-trip all miss.
bf::GraphSnapshot MakeFullFieldSnapshot() {
  bf::GraphSnapshot s;
  // 3 vertices; [2, V) is the airport region, so vertex 2 is an airport.
  s.first_airport_vertex = 2;
  s.coords = {bf::Coordinate{40.0, -73.0}, bf::Coordinate{34.0, -118.0},
              bf::Coordinate{51.4700, -0.4543}};
  // CSR: v0 -> v1 (airway 1) and v0 -> v2 (DCT), v1 has none, v2 has none.
  s.offsets = {0, 2, 2, 2};
  bf::GraphEdge e1;
  e1.to = 1;
  e1.distance_nm = 10.0f;
  e1.airway_id = 1;
  e1.base_fl = 8000;
  e1.top_fl = 24000;
  e1.level = bf::AirwayLevel::kHigh;
  bf::GraphEdge e2;
  e2.to = 2;
  e2.distance_nm = 250.0f;
  e2.airway_id = 0;  // DCT
  e2.base_fl = 0;
  e2.top_fl = 0;
  e2.level = bf::AirwayLevel::kLow;
  s.edges = {e1, e2};
  s.has_outbound = {1, 0, 0};
  s.has_inbound = {0, 1, 1};
  s.idents = {bf::FixedIdent::FromParts("AAAAA", "K1"), bf::FixedIdent::FromParts("BBBBB", "K2"),
              bf::FixedIdent::FromParts("EGLL", "EG")};
  s.kinds = {bf::WaypointKind::kFix, bf::WaypointKind::kVor, bf::WaypointKind::kDme};
  s.airway_names = {"DCT", "J80"};
  // One airport elevation for vertex 2.
  s.airport_elevations_ft = {83};
  // A non-empty MSA sector: center fix + one arc, so center/arc fields round-trip.
  bf::MsaSector msa;
  msa.center = bf::Ident("LON", "EG");
  msa.airport_icao = "EGLL";
  bf::MsaArc arc;
  arc.bearing_from = 90;
  arc.alt_100ft = 25;
  arc.radius_nm = 12;
  msa.arcs = {arc};
  s.msa = {msa};
  // One non-zero MORA cell at (lat=40, lon=-73): 3500 FL.
  s.mora.SetCell(40, -73, 3500);
  return s;
}

// Semantic round-trip: Encode a known-value snapshot, decode it, and assert
// every field against its known value -- not just structural equality. This is
// what catches an Encode/Decode mismatch that preserves wire sizes and byte
// counts (a same-width field swap or a wrong-type reinterpretation), which the
// Wire static_asserts, the trailing-bytes guard, and a self-comparing round-trip
// all fail to detect.
TEST_CASE("graph codec: every field survives the round-trip with its exact value", "[unit][bfdb]") {
  std::vector<uint8_t> body, pool;
  EncodeSnapshot(MakeFullFieldSnapshot(), &body, &pool);
  bf::Result<bf::GraphSnapshot> r = bf::GraphCodec::Decode(body, pool);
  REQUIRE(r);
  const bf::GraphSnapshot& d = r.value();

  // Header ints: v, e, airway_count, msa_count, first_airport_vertex.
  REQUIRE(d.coords.size() == 3);
  REQUIRE(d.edges.size() == 2);
  REQUIRE(d.airway_names.size() == 2);
  REQUIRE(d.msa.size() == 1);
  CHECK(d.first_airport_vertex == 2);

  // Vertex records: coord + ident refs + flags + kind, one per vertex.
  REQUIRE(d.coords.size() == 3);
  CHECK(d.coords[0].latitude == 40.0);
  CHECK(d.coords[0].longitude == -73.0);
  CHECK(d.coords[1].latitude == 34.0);
  CHECK(d.coords[1].longitude == -118.0);
  CHECK(d.coords[2].latitude == 51.47);
  CHECK(d.coords[2].longitude == -0.4543);
  REQUIRE(d.idents.size() == 3);
  CHECK(d.idents[0].IdentView() == "AAAAA");
  CHECK(d.idents[0].Arinc424IcaoCodeView() == "K1");
  CHECK(d.idents[1].IdentView() == "BBBBB");
  CHECK(d.idents[1].Arinc424IcaoCodeView() == "K2");
  CHECK(d.idents[2].IdentView() == "EGLL");
  CHECK(d.idents[2].Arinc424IcaoCodeView() == "EG");
  REQUIRE(d.has_outbound.size() == 3);
  CHECK(d.has_outbound[0] == 1);
  CHECK(d.has_outbound[1] == 0);
  CHECK(d.has_outbound[2] == 0);
  REQUIRE(d.has_inbound.size() == 3);
  CHECK(d.has_inbound[0] == 0);
  CHECK(d.has_inbound[1] == 1);
  CHECK(d.has_inbound[2] == 1);
  REQUIRE(d.kinds.size() == 3);
  CHECK(d.kinds[0] == bf::WaypointKind::kFix);
  CHECK(d.kinds[1] == bf::WaypointKind::kVor);
  CHECK(d.kinds[2] == bf::WaypointKind::kDme);

  // Airport records: one elevation per airport vertex, in vertex order.
  REQUIRE(d.airport_elevations_ft.size() == 1);
  CHECK(d.airport_elevations_ft[0] == 83);

  // CSR offsets: exact values.
  REQUIRE(d.offsets.size() == 4);
  CHECK(d.offsets[0] == 0);
  CHECK(d.offsets[1] == 2);
  CHECK(d.offsets[2] == 2);
  CHECK(d.offsets[3] == 2);

  // Edges: every field, including the float (exact -- the cache is F32, so a
  // round-trip must be bit-identical, not approximate).
  REQUIRE(d.edges.size() == 2);
  CHECK(d.edges[0].to == 1);
  CHECK(d.edges[0].distance_nm == 10.0f);
  CHECK(d.edges[0].airway_id == 1);
  CHECK(d.edges[0].base_fl == 8000);
  CHECK(d.edges[0].top_fl == 24000);
  CHECK(d.edges[0].level == bf::AirwayLevel::kHigh);
  CHECK(d.edges[1].to == 2);
  CHECK(d.edges[1].distance_nm == 250.0f);
  CHECK(d.edges[1].airway_id == 0);
  CHECK(d.edges[1].base_fl == 0);
  CHECK(d.edges[1].top_fl == 0);
  CHECK(d.edges[1].level == bf::AirwayLevel::kLow);

  // Airways: exact names.
  CHECK(d.airway_names[0] == "DCT");
  CHECK(d.airway_names[1] == "J80");

  // MSA sectors: center ident/region, airport ICAO, arc fields.
  REQUIRE(d.msa.size() == 1);
  CHECK(d.msa[0].center.ident == "LON");
  CHECK(d.msa[0].center.arinc424_icao_code == "EG");
  CHECK(d.msa[0].airport_icao == "EGLL");
  REQUIRE(d.msa[0].arcs.size() == 1);
  CHECK(d.msa[0].arcs[0].bearing_from == 90);
  CHECK(d.msa[0].arcs[0].alt_100ft == 25);
  CHECK(d.msa[0].arcs[0].radius_nm == 12);

  // MORA: the one populated cell must round-trip; the rest stay 0.
  CHECK(d.mora.MoraAt(bf::Coordinate{40.0, -73.0}) == 3500);
  CHECK(d.mora.MoraAt(bf::Coordinate{41.0, -74.0}) == 0);
}

}  // namespace

TEST_CASE("graph codec: a corrupt snapshot is rejected on encode, a corrupt section on decode",
          "[unit][bfdb]") {
  // Control: the valid snapshot round-trips through Encode/Decode.
  {
    std::vector<uint8_t> body, pool;
    EncodeSnapshot(MakeValidSnapshot(), &body, &pool);
    bf::Result<bf::GraphSnapshot> r = bf::GraphCodec::Decode(body, pool);
    REQUIRE(r);
    CHECK(r.value().coords.size() == 2);
  }

  // Encode-side: each case corrupts exactly one semantic field of an otherwise
  // valid snapshot while keeping array sizes consistent. Encode now validates
  // these invariants (not just sizes), so it rejects the snapshot with
  // kSerializationError before any section is written -- a GraphBuilder bug surfaces at
  // `bf build` instead of only as kCacheCorrupt on a later round-trip.
  auto expect_encode_rejects = [](const bf::GraphSnapshot& s) {
    std::vector<uint8_t> body, pool_blob;
    bf::ByteWriter w(body);
    bf::StringPool pool;
    bf::Result<void> enc = bf::GraphCodec::Encode(s, w, pool);
    CHECK_FALSE(enc);
    if (!enc) {
      CHECK(enc.error().code == bf::ErrorCode::kSerializationError);
    }
  };
  SECTION("encode rejects edge target vertex out of range") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.edges[0].to = 5;  // >= vertex count (2)
    expect_encode_rejects(s);
  }
  SECTION("encode rejects edge target vertex negative") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.edges[0].to = -1;
    expect_encode_rejects(s);
  }
  SECTION("encode rejects edge airway-name index out of range") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.edges[0].airway_id = 7;  // >= airway_names count (1)
    expect_encode_rejects(s);
  }
  SECTION("encode rejects edge airway level enum out of range") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.edges[0].level = static_cast<bf::AirwayLevel>(99);
    expect_encode_rejects(s);
  }
  SECTION("encode rejects vertex kind enum out of range") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.kinds[0] = static_cast<bf::WaypointKind>(99);
    expect_encode_rejects(s);
  }
  SECTION("encode rejects CSR offsets not ending at the edge count") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.offsets = {0, 1, 5};  // back != e (1)
    expect_encode_rejects(s);
  }
  SECTION("encode rejects CSR offsets that are not monotonic") {
    bf::GraphSnapshot s = MakeValidSnapshot();
    s.edges.clear();        // e = 0, so back must be 0 to span correctly
    s.offsets = {0, 2, 0};  // spans [0,0] at the ends but dips downward: not monotonic
    expect_encode_rejects(s);
  }

  // Decode-side: Encode now guarantees a well-formed section, so the realistic
  // threat is a valid encoding byte-corrupted on disk (bit-flip / half-write).
  // Decode must still reject it through Result (kCacheCorrupt), never index the
  // graph out of range.
  SECTION("decode rejects trailing bytes after a valid section") {
    std::vector<uint8_t> body, pool;
    EncodeSnapshot(MakeValidSnapshot(), &body, &pool);
    body.push_back(0);  // one extra byte the decoder should not have to read
    bf::Result<bf::GraphSnapshot> r = bf::GraphCodec::Decode(body, pool);
    CHECK_FALSE(r);
    if (!r) {
      CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    }
  }
  SECTION("decode rejects a byte-corrupted edge target") {
    // Offset of the first edge's `to` field in a MakeValidSnapshot() encoding:
    // header (5*U32 = 20) + 2 vertex records (34 B each) + 0 airport records +
    // 3 offsets (I32 each) = 100. The wire layout is fixed (static_asserts in
    // graph_codec.cc guard each record size); if it changes, this and Encode's
    // layout comment move together.
    constexpr size_t kFirstEdgeToOffset = 5 * 4 + 2 * 34 + 0 * 4 + 3 * 4;
    static_assert(kFirstEdgeToOffset == 100, "graph wire layout drifted");
    std::vector<uint8_t> body, pool;
    EncodeSnapshot(MakeValidSnapshot(), &body, &pool);
    REQUIRE(body.size() > kFirstEdgeToOffset + 3);
    // Overwrite the little-endian I32 `to` with 0x7FFFFFFF (>= vertex count 2).
    body[kFirstEdgeToOffset] = 0xFF;
    body[kFirstEdgeToOffset + 1] = 0xFF;
    body[kFirstEdgeToOffset + 2] = 0xFF;
    body[kFirstEdgeToOffset + 3] = 0x7F;
    bf::Result<bf::GraphSnapshot> r = bf::GraphCodec::Decode(body, pool);
    CHECK_FALSE(r);
    if (!r) {
      CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    }
  }
}

// The L2 hardening completed the Decode byte-count fuse: the combined-minimum
// check now also accounts for the fixed CSR offsets row ((v+1) I32s) and the
// MORA grid (kLatCount*kLonCount int16 cells), both unconditionally present in
// the body, so a header whose sections collectively demand more bytes than the
// body holds is rejected. This forges exactly that: a header whose per-count
// bounds ALL pass (each section individually fits the remaining bytes) but whose
// combined minimum -- once the CSR row and MORA grid are summed in -- exceeds the
// bytes actually present. Decode must return Err(kCacheCorrupt); ByteReader still
// bounds-checks every read, so even if the fuse were absent the decode degrades
// to the same Err rather than reading out of bounds, but the test locks the
// rejection contract the fuse guarantees.
TEST_CASE("graph codec: header passing per-count but over-combined is rejected (L2 fuse)",
          "[unit][bfdb]") {
  std::vector<uint8_t> body, pool;
  EncodeSnapshot(MakeValidSnapshot(), &body, &pool);
  REQUIRE_FALSE(body.empty());

  // Forge the vertex count `v` (first U32, little-endian). The valid snapshot has
  // v=2; we raise it to 2500, which still satisfies every per-count bound for a
  // body that contains a full MORA grid (kLatCount*kLonCount cells ~= 64800, so
  // the body is ~129 KiB and the per-count ceiling for v is ~3814), yet the
  // combined minimum (v*34 + (v-2)*4 airport + (v+1)*4 CSR + MORA + ...) far
  // exceeds the ~129 KiB actually present. Only the count field is flipped; the
  // body bytes are left exactly as encoded, so this is purely a "counts lie about
  // the body" forgery that the combined fuse must catch.
  constexpr uint32_t kForgedV = 2500;
  body[0] = static_cast<uint8_t>(kForgedV & 0xFF);
  body[1] = static_cast<uint8_t>((kForgedV >> 8) & 0xFF);
  body[2] = static_cast<uint8_t>((kForgedV >> 16) & 0xFF);
  body[3] = static_cast<uint8_t>((kForgedV >> 24) & 0xFF);

  bf::Result<bf::GraphSnapshot> r = bf::GraphCodec::Decode(body, pool);
  CHECK_FALSE(r);
  if (!r) {
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
  }
}

TEST_CASE("bfdb: a cached route matches the freshly built route", "[integration][bfdb]") {
  const std::string dir = EnsureXPlane12();
  bf::Result<bf::NavDatabase> direct = bf::NavDatabase::Open(dir);
  if (!direct) {
    SKIP("navigation data not found in '" << dir << "' (set BRAVOFINDER_NAVDATA)");
  }
  const std::string path = TempBfdb("roundtrip");
  // The unified file carries graph + CIFP + detail, so the cached path resolves
  // the same procedures (SID/STAR) the direct path parses from CIFP files.
  REQUIRE(direct.value().WriteUnified(path));

  bf::Result<bf::NavDatabase> cached = bf::NavDatabase::OpenCached(path);
  REQUIRE(cached);

  // Same query on both databases must yield identical routes.
  const bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  bf::Result<std::vector<bf::Route>> a = direct.value().FindRoutes(req);
  bf::Result<std::vector<bf::Route>> b = cached.value().FindRoutes(req);
  REQUIRE(a);
  REQUIRE(b);
  REQUIRE_FALSE(a.value().empty());
  REQUIRE(a.value().size() == b.value().size());

  const bf::Route& ra = a.value().front();
  const bf::Route& rb = b.value().front();
  CHECK(ra.route_string == rb.route_string);
  CHECK(ra.sid == rb.sid);
  CHECK(ra.star == rb.star);
  CHECK(ra.legs.size() == rb.legs.size());
  // Distance stored as float in the cache; allow a small tolerance.
  CHECK(rb.total_distance_nm == Catch::Approx(ra.total_distance_nm).margin(0.01));
  for (size_t i = 0; i < ra.legs.size(); ++i) {
    CHECK(ra.legs[i].from == rb.legs[i].from);
    CHECK(ra.legs[i].to == rb.legs[i].to);
    CHECK(ra.legs[i].via == rb.legs[i].via);
  }

  std::remove(path.c_str());
}

TEST_CASE("bfdb: the container header preserves AIRAC provenance", "[integration][bfdb]") {
  const std::string path = BuildCache("meta");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::UnifiedData> u = bf::UnifiedCache::Open(path);
  REQUIRE(u);
  // cycle 2601 for the test dataset; should be non-zero and the graph non-empty.
  CHECK(u.value().header.cycle != 0);
  CHECK_FALSE(u.value().graph.coords.empty());
  CHECK(u.value().graph.offsets.size() == u.value().graph.coords.size() + 1);
  std::remove(path.c_str());
}

TEST_CASE("bfdb: the cache preserves waypoint kinds and airport elevations",
          "[integration][bfdb]") {
  const std::string path = BuildCache("fields");
  if (path.empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<bf::UnifiedData> u = bf::UnifiedCache::Open(path);
  REQUIRE(u);
  const bf::GraphSnapshot& snapshot = u.value().graph;

  // Per-vertex kinds are present for every vertex.
  REQUIRE(snapshot.kinds.size() == snapshot.coords.size());
  // Real AIRAC data has a mix of fixes and navaids, so at least one vertex must
  // be a navaid kind -- proving the field is populated, not defaulted to kFix.
  bool has_navaid = false;
  for (bf::WaypointKind k : snapshot.kinds) {
    if (k == bf::WaypointKind::kVor || k == bf::WaypointKind::kNdb || k == bf::WaypointKind::kDme) {
      has_navaid = true;
      break;
    }
  }
  CHECK(has_navaid);

  // Airport elevations: one per airport vertex, and at least one non-zero (most
  // airports sit above sea level), proving elevation survived the round-trip.
  const size_t airport_count =
      snapshot.coords.size() - static_cast<size_t>(snapshot.first_airport_vertex);
  REQUIRE(snapshot.airport_elevations_ft.size() == airport_count);
  bool has_nonzero_elev = false;
  for (int e : snapshot.airport_elevations_ft) {
    if (e != 0) {
      has_nonzero_elev = true;
      break;
    }
  }
  CHECK(has_nonzero_elev);

  // Per-vertex airway-membership flags: both directions are present for every
  // vertex (the hard, data-independent invariant). Whether real data happens to
  // contain an inbound-only vertex (has_inbound && !has_outbound -- a STAR entry
  // gate reached only via a forward-only airway, e.g. ABBEY) is cycle-dependent:
  // most cycles have one, but a cycle lacking any forward-only dead end would
  // have none. So the existence check is a soft WARN, not a hard CHECK, to stay
  // green across cycles; the data-independent round-trip of an inbound-only
  // vertex is pinned in the unified_cache_test "inbound-only vertex flags
  // survive the round-trip" case.
  REQUIRE(snapshot.has_outbound.size() == snapshot.coords.size());
  REQUIRE(snapshot.has_inbound.size() == snapshot.coords.size());
  size_t inbound_only_count = 0;
  for (size_t i = 0; i < snapshot.coords.size(); ++i) {
    if (snapshot.has_inbound[i] && !snapshot.has_outbound[i]) {
      ++inbound_only_count;
    }
  }
  if (inbound_only_count == 0) {
    WARN(
        "no inbound-only vertex in this AIRAC cycle (forward-only dead-ends "
        "absent); the bit1 round-trip is covered by the data-independent "
        "unified_cache_test case");
  }

  std::remove(path.c_str());
}

TEST_CASE("bfdb: an airport without procedures still routes via the cache", "[integration][bfdb]") {
  const std::string dir = EnsureXPlane12();
  const std::string path = BuildCache("nocifp");
  if (path.empty()) {
    SKIP("navigation data not found in '" << dir << "'");
  }
  bf::Result<bf::NavDatabase> cached = bf::NavDatabase::OpenCached(path);
  REQUIRE(cached);
  // KIKR has no CIFP file; the airport should still connect via DCT fallback.
  bf::Result<std::vector<bf::Route>> routes =
      cached.value().FindRoutes(MakeRequest("KIKR", "KLAX"));
  // Either a route is found (DCT fallback works) or a clean no-route error; the
  // point is that OpenCached does not crash on missing CIFP.
  if (routes) {
    CHECK_FALSE(routes.value().empty());
  } else {
    CHECK(routes.error().code != bf::ErrorCode::kUnknown);
  }
  std::remove(path.c_str());
}

TEST_CASE("bfdb: a corrupt or missing cache is rejected cleanly", "[unit][bfdb]") {
  // Missing file.
  {
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(TempBfdb("does_not_exist"));
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kDataMissing);
  }
  // Bad magic.
  {
    const std::string path = TempBfdb("badmagic");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "NOPEnot a real bfdb file at all";
    f.close();
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
  // Right magic, wrong version.
  {
    const std::string path = TempBfdb("badver");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "BFDB";
    const uint32_t bad_version = 0xDEADBEEF;
    f.write(reinterpret_cast<const char*>(&bad_version), sizeof(bad_version));
    f.close();
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kFormatMismatch);
    std::remove(path.c_str());
  }
  // Truncated (only magic).
  {
    const std::string path = TempBfdb("trunc");
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    f << "BF";
    f.close();
    bf::Result<bf::UnifiedData> r = bf::UnifiedCache::Open(path);
    CHECK_FALSE(r);
    CHECK(r.error().code == bf::ErrorCode::kCacheCorrupt);
    std::remove(path.c_str());
  }
}
