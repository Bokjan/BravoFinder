// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/cache/graph_codec.h"

#include <cstdint>

#include "io/cache/byte_io.h"
#include "io/cache/graph_snapshot.h"

namespace bf {

namespace {

// The vertex-record flags byte and the WaypointKind are each serialized as a
// single U8; guard that the kind still fits, so adding enumerators past 255
// fails to compile rather than silently truncating on write.
static_assert(static_cast<int>(WaypointKind::kOther) < 256,
              "WaypointKind no longer fits in the U8 vertex-record field");

// Wire-layout declarators for the fixed-length records in this section. These
// exist ONLY so `sizeof` is the single source of truth for the corruption fuses
// below (count_fits / min_body_bytes / per-sector arc bound): the wire format is
// packed with no padding, while the in-memory types have natural alignment
// (GraphEdge is 16 B in memory, 15 B on the wire) and use FixedIdent / std::string
// where the wire stores (off,len) ref pairs. So these are deliberately distinct
// from the in-memory structs. Encode/Decode stay field-by-field via ByteWriter --
// never instantiate, memcpy, or take a member address of these (packed-member
// access is UB; #pragma pack(push,1) is portable across MSVC/GCC/Clang for sizeof).
#pragma pack(push, 1)
struct WireVertex {
  double lat, lon;
  uint32_t ident_io, ident_il, region_io, region_rl;
  uint8_t flags, kind;
};
struct WireEdge {
  int32_t to;
  float distance_nm;
  uint16_t airway_id;
  int16_t base_fl, top_fl;
  uint8_t level;
};
struct WireAirwayRef {
  uint32_t name_off, name_len;
};
struct WireMsaHeader {
  uint32_t cio, cil, cro, crl, aio, ail, arc_count;
};
struct WireMsaArc {
  int32_t bearing_from, alt_100ft, radius_nm;
};
#pragma pack(pop)

constexpr size_t kVertexRecordSize = sizeof(WireVertex);  // 34
constexpr size_t kAirportRecordSize = sizeof(int32_t);    // 4  (single I32 elevation)
constexpr size_t kEdgeRecordSize = sizeof(WireEdge);      // 15
constexpr size_t kAirwayRefSize = sizeof(WireAirwayRef);  // 8
constexpr size_t kMsaHeaderSize = sizeof(WireMsaHeader);  // 28 (fixed prefix of a variable record)
constexpr size_t kMsaArcSize = sizeof(WireMsaArc);        // 12
static_assert(kVertexRecordSize == 34, "vertex wire layout drifted");
static_assert(kEdgeRecordSize == 15, "edge wire layout drifted");
static_assert(kAirwayRefSize == 8, "airway-ref wire layout drifted");
static_assert(kMsaHeaderSize == 28, "msa-header wire layout drifted");
static_assert(kMsaArcSize == 12, "msa-arc wire layout drifted");

}  // namespace

// The graph section body layout (all string refs point into the shared global
// pool):
//   header ints : U32 v, U32 e, U32 airway_count, U32 msa_count,
//                 U32 first_airport_vertex
//   vertex records [v] : F64 lat, F64 lon, U32 ident_off/len, U32 region_off/len,
//                        U8 flags (bit0 = has_outbound, bit1 = has_inbound), U8 kind
//   airport records [v - first_airport_vertex] : I32 elevation_ft
//   offsets [v + 1] : I32
//   edges [e] : I32 to, F32 distance_nm, U16 airway_id, I16 base_fl, I16 top_fl,
//               U8 level (AirwayLevel)
//   airways [airway_count] : U32 name_off, U32 name_len
//   mora : kLatCount * kLonCount I16 cells
//   msa [msa_count] : U32 center ident off/len, U32 center region off/len,
//                     U32 airport off/len, U32 arc_count, then per arc
//                     I32 bearing_from, I32 alt_100ft, I32 radius_nm
Result<void> GraphCodec::Encode(const GraphSnapshot& snapshot, ByteWriter& w, StringPool& pool) {
  const size_t v = snapshot.coords.size();
  const size_t e = snapshot.edges.size();
  if (snapshot.airway_names.size() > 0xFFFF) {
    return Result<void>::Err(
        Error(ErrorCode::kParseError, "too many airway names to serialize (> 65535)"));
  }
  if (snapshot.offsets.size() != v + 1 || snapshot.idents.size() != v ||
      snapshot.has_outbound.size() != v || snapshot.has_inbound.size() != v ||
      snapshot.kinds.size() != v) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "inconsistent snapshot array sizes"));
  }
  const size_t airport_count = v - static_cast<size_t>(snapshot.first_airport_vertex);
  if (snapshot.first_airport_vertex < 0 || static_cast<size_t>(snapshot.first_airport_vertex) > v ||
      snapshot.airport_elevations_ft.size() != airport_count) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "inconsistent airport array size"));
  }

  // Section counts, up front so Decode can allocate before reading bodies.
  w.U32(static_cast<uint32_t>(v));
  w.U32(static_cast<uint32_t>(e));
  w.U32(static_cast<uint32_t>(snapshot.airway_names.size()));
  w.U32(static_cast<uint32_t>(snapshot.msa.size()));
  w.U32(static_cast<uint32_t>(snapshot.first_airport_vertex));

  // Vertex records: one self-contained record per vertex, gathering all
  // per-vertex attributes (position, ident, a flags byte, and the point kind).
  // Adding a new per-vertex field means one more field in this record -- no new
  // parallel array, no separate section. Airports (vertices
  // [first_airport_vertex, V)) carry their airport-only attributes in a separate
  // record section below.
  for (size_t i = 0; i < v; ++i) {
    const Coordinate& c = snapshot.coords[i];
    w.F64(c.latitude);
    w.F64(c.longitude);
    const FixedIdent& id = snapshot.idents[i];
    const auto ir = pool.Add(std::string(id.IdentView()));
    const auto rr = pool.Add(std::string(id.RegionView()));
    w.U32(ir.first);
    w.U32(ir.second);
    w.U32(rr.first);
    w.U32(rr.second);
    uint8_t flags = 0;
    if (snapshot.has_outbound[i]) {
      flags |= 0x01;
    }
    if (snapshot.has_inbound[i]) {
      flags |= 0x02;
    }
    w.U8(flags);
    w.U8(static_cast<uint8_t>(snapshot.kinds[i]));
  }

  // Airport records: one per airport vertex, in vertex order.
  for (int elev : snapshot.airport_elevations_ft) {
    w.I32(elev);
  }

  // CSR graph structure (not per-vertex attributes, kept as flat arrays):
  for (int off : snapshot.offsets) {
    w.I32(off);
  }
  for (const GraphEdge& ed : snapshot.edges) {
    w.I32(ed.to);
    w.F32(ed.distance_nm);
    w.U16(ed.airway_id);
    w.I16(ed.base_fl);
    w.I16(ed.top_fl);
    w.U8(static_cast<uint8_t>(ed.level));
  }
  for (const std::string& name : snapshot.airway_names) {
    const auto nr = pool.Add(name);
    w.U32(nr.first);
    w.U32(nr.second);
  }
  for (int16_t cell : snapshot.mora.cells()) {
    w.I16(cell);
  }
  for (const MsaSector& s : snapshot.msa) {
    const auto ci = pool.Add(s.center.ident);
    const auto cr = pool.Add(s.center.region);
    const auto ai = pool.Add(s.airport_icao);
    w.U32(ci.first);
    w.U32(ci.second);
    w.U32(cr.first);
    w.U32(cr.second);
    w.U32(ai.first);
    w.U32(ai.second);
    w.U32(static_cast<uint32_t>(s.arcs.size()));
    for (const MsaArc& arc : s.arcs) {
      w.I32(arc.bearing_from);
      w.I32(arc.alt_100ft);
      w.I32(arc.radius_nm);
    }
  }
  return Result<void>::Ok();
}

Result<GraphSnapshot> GraphCodec::Decode(std::span<const uint8_t> body,
                                         std::span<const uint8_t> pool) {
  ByteReader r(body);

  auto bad = [](const char* why) {
    return Result<GraphSnapshot>::Err(
        Error(ErrorCode::kCacheCorrupt, std::string(why) + "; run bf build to regenerate"));
  };

  GraphSnapshot snapshot;
  const uint32_t v = r.U32();
  const uint32_t e = r.U32();
  const uint32_t airway_count = r.U32();
  const uint32_t msa_count = r.U32();
  snapshot.first_airport_vertex = static_cast<int>(r.U32());

  // Sanity-check the counts against the bytes actually present BEFORE any
  // resize, so a corrupt or forged section cannot trigger a huge allocation (and
  // a bad_alloc/length_error that would bypass Result). Each count must fit in
  // the remaining bytes at its minimum on-disk element size; this is a necessary
  // condition, not an exact one -- a fuse, not a full validator. The per-record
  // sizes come from the packed WireRecord declarators above (kVertexRecordSize
  // etc.), so adding a field fails the static_assert rather than silently leaving
  // a stale literal here. On-disk sizes: vertex record kVertexRecordSize, airport
  // record kAirportRecordSize, offsets 4 B, GraphEdge kEdgeRecordSize, airway ref
  // kAirwayRefSize, MSA sector at least kMsaHeaderSize (EXCLUDING its variable-
  // length arc array -- each sector's arcs are separately fused against the
  // then-remaining bytes in the MSA read loop below).
  //
  // first_airport_vertex must lie in [0, v]; the airport record section then
  // holds (v - first_airport_vertex) elevations. An out-of-range value is
  // rejected here rather than clamped, so the airport count below is trustworthy.
  if (!r.ok() || snapshot.first_airport_vertex < 0 ||
      static_cast<uint32_t>(snapshot.first_airport_vertex) > v) {
    return bad("corrupt .bfdb: first_airport_vertex out of range");
  }
  const uint32_t airport_count = v - static_cast<uint32_t>(snapshot.first_airport_vertex);
  const size_t avail = r.remaining();
  auto count_fits = [&](uint32_t count, size_t per_elem) {
    return static_cast<size_t>(count) <= avail / per_elem;
  };
  if (!count_fits(v, kVertexRecordSize) || !count_fits(airport_count, kAirportRecordSize) ||
      !count_fits(e, kEdgeRecordSize) || !count_fits(airway_count, kAirwayRefSize) ||
      !count_fits(msa_count, kMsaHeaderSize)) {
    return bad("corrupt .bfdb: section counts exceed section size");
  }
  // Each fuse above bounds one count against the whole remaining section without
  // debiting the others, so a crafted header could pass all five yet demand far
  // more than `avail` in total (each resize below allocates independently). Cap
  // the combined minimum too. Products fit in size_t: every count is a uint32
  // and per_elem is tiny, on a 64-bit size_t.
  const size_t min_body_bytes = static_cast<size_t>(v) * kVertexRecordSize +
                                static_cast<size_t>(airport_count) * kAirportRecordSize +
                                static_cast<size_t>(e) * kEdgeRecordSize +
                                static_cast<size_t>(airway_count) * kAirwayRefSize +
                                static_cast<size_t>(msa_count) * kMsaHeaderSize;
  if (min_body_bytes > avail) {
    return bad("corrupt .bfdb: combined section counts exceed section size");
  }

  // Vertex records: coord + ident refs + flags + kind, one per vertex. Ident
  // refs are resolved against the global string pool after the body is read.
  struct IdentRef {
    uint32_t io, il, ro, rl;
  };
  snapshot.coords.resize(v);
  snapshot.has_outbound.assign(v, 0);
  snapshot.has_inbound.assign(v, 0);
  snapshot.kinds.resize(v);
  std::vector<IdentRef> ident_refs(v);
  for (uint32_t i = 0; i < v; ++i) {
    snapshot.coords[i].latitude = r.F64();
    snapshot.coords[i].longitude = r.F64();
    ident_refs[i].io = r.U32();
    ident_refs[i].il = r.U32();
    ident_refs[i].ro = r.U32();
    ident_refs[i].rl = r.U32();
    const uint8_t flags = r.U8();
    snapshot.has_outbound[i] = (flags & 0x01) != 0 ? 1 : 0;
    snapshot.has_inbound[i] = (flags & 0x02) != 0 ? 1 : 0;
    snapshot.kinds[i] = static_cast<WaypointKind>(r.U8());
  }
  // Airport records: elevation per airport vertex, in vertex order.
  snapshot.airport_elevations_ft.resize(airport_count);
  for (uint32_t i = 0; i < airport_count; ++i) {
    snapshot.airport_elevations_ft[i] = r.I32();
  }
  // offsets
  snapshot.offsets.resize(static_cast<size_t>(v) + 1);
  r.I32Span(snapshot.offsets.data(), static_cast<size_t>(v) + 1);
  // edges
  snapshot.edges.resize(e);
  for (uint32_t i = 0; i < e; ++i) {
    GraphEdge ed;
    ed.to = r.I32();
    ed.distance_nm = r.F32();
    ed.airway_id = r.U16();
    ed.base_fl = r.I16();
    ed.top_fl = r.I16();
    ed.level = static_cast<AirwayLevel>(r.U8());
    snapshot.edges[i] = ed;
  }
  // airways
  struct NameRef {
    uint32_t o, l;
  };
  std::vector<NameRef> airway_refs(airway_count);
  for (uint32_t i = 0; i < airway_count; ++i) {
    airway_refs[i].o = r.U32();
    airway_refs[i].l = r.U32();
  }
  // mora
  std::vector<int16_t> cells(static_cast<size_t>(MoraGrid::kLatCount) * MoraGrid::kLonCount);
  r.I16Span(cells.data(), cells.size());
  // msa
  struct MsaRef {
    uint32_t cio, cil, cro, crl, aio, ail;
    std::vector<MsaArc> arcs;
  };
  std::vector<MsaRef> msa_refs(msa_count);
  for (uint32_t i = 0; i < msa_count; ++i) {
    MsaRef& m = msa_refs[i];
    m.cio = r.U32();
    m.cil = r.U32();
    m.cro = r.U32();
    m.crl = r.U32();
    m.aio = r.U32();
    m.ail = r.U32();
    const uint32_t arc_count = r.U32();
    // Fuse against the CURRENTLY remaining bytes (shrinks as prior sectors are
    // consumed), each arc being kMsaArcSize. A necessary, per-sector bound; the
    // subsequent reads still degrade safely via r.ok() on any residual mismatch.
    if (!r.ok() || arc_count > r.remaining() / kMsaArcSize) {
      return bad("corrupt .bfdb msa section");
    }
    m.arcs.resize(arc_count);
    for (uint32_t a = 0; a < arc_count; ++a) {
      m.arcs[a].bearing_from = r.I32();
      m.arcs[a].alt_100ft = r.I32();
      m.arcs[a].radius_nm = r.I32();
    }
  }
  if (!r.ok()) {
    return bad("corrupt .bfdb graph section");
  }
  // A well-formed section is consumed exactly; leftover bytes mean a count was
  // under-read (a corrupt/half-written same-version file), so reject it.
  if (r.remaining() != 0) {
    return bad("corrupt .bfdb: graph section has trailing bytes");
  }

  // Semantic validation of the CSR structure and in-record references. The
  // byte-level fuses above bound the section SIZE, but not the VALUES inside it:
  // a bit-flip or half-write in a same-format-version file can pass every check
  // so far and then, on the A*/Yen hot path, read out of bounds -- the NavGraph
  // accessors (CoordOf/EdgesBegin/EdgesEnd) and GraphBuilder::AirwayName all use
  // raw unchecked indexing. Validate here so a corrupt cache fails through Result
  // (kCacheCorrupt) instead of undefined behavior.
  //
  // CSR offsets must start at 0, end at the edge count, and be non-decreasing;
  // that also transitively bounds every offset to [0, e], keeping EdgesBegin/End
  // in range.
  if (snapshot.offsets.front() != 0 || snapshot.offsets.back() != static_cast<int>(e)) {
    return bad("corrupt .bfdb: CSR offsets do not span [0, edge count]");
  }
  for (uint32_t i = 0; i < v; ++i) {
    if (snapshot.offsets[i] > snapshot.offsets[i + 1]) {
      return bad("corrupt .bfdb: CSR offsets are not monotonic");
    }
  }
  // Each edge's target vertex, airway-name index, and level enum must be in range
  // (airway_id == 0 is DCT, but it still indexes airway_names[0], so a file with
  // edges must carry at least that name).
  for (const GraphEdge& ed : snapshot.edges) {
    if (ed.to < 0 || static_cast<uint32_t>(ed.to) >= v) {
      return bad("corrupt .bfdb: edge target vertex out of range");
    }
    if (ed.airway_id >= airway_count) {
      return bad("corrupt .bfdb: edge airway-name index out of range");
    }
    if (static_cast<uint8_t>(ed.level) > static_cast<uint8_t>(AirwayLevel::kBoth)) {
      return bad("corrupt .bfdb: edge airway level out of range");
    }
  }
  // Per-vertex kind enum must be a valid WaypointKind.
  for (const WaypointKind k : snapshot.kinds) {
    if (static_cast<uint8_t>(k) > static_cast<uint8_t>(WaypointKind::kOther)) {
      return bad("corrupt .bfdb: vertex kind out of range");
    }
  }

  // Resolve all string references against the global pool.
  bool refs_ok = true;
  snapshot.idents.resize(v);
  for (uint32_t i = 0; i < v; ++i) {
    const IdentRef& ir = ident_refs[i];
    const std::string id = ResolveRef(pool, ir.io, ir.il, refs_ok);
    const std::string reg = ResolveRef(pool, ir.ro, ir.rl, refs_ok);
    snapshot.idents[i] = FixedIdent::FromParts(id, reg);
  }
  snapshot.airway_names.resize(airway_count);
  for (uint32_t i = 0; i < airway_count; ++i) {
    snapshot.airway_names[i] = ResolveRef(pool, airway_refs[i].o, airway_refs[i].l, refs_ok);
  }
  snapshot.mora = MoraGrid::FromCells(std::move(cells));
  snapshot.msa.resize(msa_count);
  for (uint32_t i = 0; i < msa_count; ++i) {
    MsaRef& m = msa_refs[i];
    snapshot.msa[i].center.ident = ResolveRef(pool, m.cio, m.cil, refs_ok);
    snapshot.msa[i].center.region = ResolveRef(pool, m.cro, m.crl, refs_ok);
    snapshot.msa[i].airport_icao = ResolveRef(pool, m.aio, m.ail, refs_ok);
    snapshot.msa[i].arcs = std::move(m.arcs);
  }
  if (!refs_ok) {
    return bad("corrupt .bfdb: field or reference out of range");
  }
  return Result<GraphSnapshot>::Ok(std::move(snapshot));
}

}  // namespace bf
