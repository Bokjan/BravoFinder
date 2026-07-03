#include "io/cache/bfdb_cache.h"

#include <cstring>
#include <fstream>
#include <unordered_map>

#include "io/cache/byte_io.h"

namespace bf {

namespace {

constexpr char kMagic[4] = {'B', 'F', 'D', 'B'};

}  // namespace

Result<void> BfdbCache::Write(const std::string& path, const BfdbImage& image) {
  const size_t v = image.coords.size();
  const size_t e = image.edges.size();
  if (image.airway_names.size() > 0xFFFF) {
    return Result<void>::Err(
        Error(ErrorCode::kParseError, "too many airway names to serialize (> 65535)"));
  }
  if (image.offsets.size() != v + 1 || image.idents.size() != v || image.on_network.size() != v ||
      image.kinds.size() != v) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "inconsistent image array sizes"));
  }
  const size_t airport_count = v - static_cast<size_t>(image.first_airport_vertex);
  if (image.first_airport_vertex < 0 || static_cast<size_t>(image.first_airport_vertex) > v ||
      image.airport_elevations_ft.size() != airport_count) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "inconsistent airport array size"));
  }

  // Sections are serialized into their own buffers first; string references are
  // accumulated into a shared pool, whose blob is appended last.
  StringPool pool;
  std::string sections;
  ByteWriter w(sections);

  // Vertex records: one self-contained record per vertex, gathering all
  // per-vertex attributes (position, ident, a flags byte, and the point kind).
  // Adding a new per-vertex field means one more field in this record -- no new
  // parallel array, no separate section. Airports (vertices
  // [first_airport_vertex, V)) carry their airport-only attributes in a separate
  // record section below.
  //   coord      : F64 lat, F64 lon
  //   ident      : U32 ident_off, U32 ident_len, U32 region_off, U32 region_len
  //   flags      : U8  (bit 0 = on_network)
  //   kind       : U8  (WaypointKind)
  for (size_t i = 0; i < v; ++i) {
    const Coordinate& c = image.coords[i];
    w.F64(c.latitude);
    w.F64(c.longitude);
    const Ident& id = image.idents[i];
    const auto ir = pool.Add(id.ident);
    const auto rr = pool.Add(id.region);
    w.U32(ir.first);
    w.U32(ir.second);
    w.U32(rr.first);
    w.U32(rr.second);
    uint8_t flags = 0;
    if (image.on_network[i]) {
      flags |= 0x01;
    }
    w.U8(flags);
    w.U8(static_cast<uint8_t>(image.kinds[i]));
  }

  // Airport records: one per airport vertex, in vertex order. Airport-only
  // attributes live here so they do not bloat the V vertex records.
  //   elevation_ft : I32
  for (int elev : image.airport_elevations_ft) {
    w.I32(elev);
  }

  // CSR graph structure (not per-vertex attributes, kept as flat arrays):
  // offsets: (V + 1) * int32
  for (int off : image.offsets) {
    w.I32(off);
  }
  // edges: E * GraphEdge (16 bytes each, field by field)
  for (const GraphEdge& ed : image.edges) {
    w.I32(ed.to);
    w.F32(ed.distance_nm);
    w.U16(ed.airway_id);
    w.I16(ed.base_fl);
    w.I16(ed.top_fl);
    w.U8(ed.flags);
  }
  // airways: count * name_ref
  for (const std::string& name : image.airway_names) {
    const auto nr = pool.Add(name);
    w.U32(nr.first);
    w.U32(nr.second);
  }
  // mora: populated implied; 64800 int16 cells
  for (int16_t cell : image.mora.cells()) {
    w.I16(cell);
  }
  // msa: count * sector
  for (const MsaSector& s : image.msa) {
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

  // Header, then sections, then the string pool blob.
  std::string out;
  ByteWriter hw(out);
  out.append(kMagic, 4);
  hw.U32(kFormatVersion);
  hw.U32(image.cycle);
  hw.U32(image.build);
  hw.U32(static_cast<uint32_t>(v));
  hw.U32(static_cast<uint32_t>(e));
  hw.U32(static_cast<uint32_t>(image.airway_names.size()));
  hw.U32(static_cast<uint32_t>(image.msa.size()));
  hw.U32(static_cast<uint32_t>(image.first_airport_vertex));
  // Header strings are stored inline (length-prefixed) rather than in the pool,
  // since the pool is a trailing section but these are read from the header.
  auto write_inline = [&](const std::string& s) {
    hw.U32(static_cast<uint32_t>(s.size()));
    out.append(s);
  };
  write_inline(image.program_semver);
  write_inline(image.source_loader);
  write_inline(image.data_dir);
  hw.U32(static_cast<uint32_t>(pool.blob().size()));

  out.append(sections);
  out.append(pool.blob());

  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f.is_open()) {
    return Result<void>::Err(
        Error(ErrorCode::kDataMissing, "cannot open .bfdb for writing: " + path));
  }
  f.write(out.data(), static_cast<std::streamsize>(out.size()));
  if (!f) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "failed writing .bfdb: " + path));
  }
  return Result<void>::Ok();
}

// Read only the fixed header region: magic, format version, cycle/build, and
// the three inline provenance strings. Deliberately mirrors the header layout
// written by Write (see the "Header, then sections..." block there) but stops
// before the vertex records. The count fields are skipped, not validated,
// since the body is never touched.
Result<BfdbHeader> BfdbCache::ReadHeader(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return Result<BfdbHeader>::Err(Error(ErrorCode::kDataMissing, "cannot open .bfdb: " + path));
  }
  // The header is small and bounded: 4 (magic) + 9*U32 (version, cycle, build,
  // v, e, airway_count, msa_count, first_airport_vertex, and each string's
  // length prefix) plus the three variable-length strings. Read a generous
  // fixed prefix; if a string length points past it, the file is treated as
  // corrupt rather than read further (a real cache's provenance strings are a
  // few dozen bytes).
  constexpr size_t kMaxHeader = 4096;
  std::string buf(kMaxHeader, '\0');
  f.read(buf.data(), static_cast<std::streamsize>(kMaxHeader));
  buf.resize(static_cast<size_t>(f.gcount()));
  if (buf.size() < 4) {
    return Result<BfdbHeader>::Err(Error(ErrorCode::kDataMissing, "truncated .bfdb: " + path));
  }

  auto bad = [&](const char* why) {
    return Result<BfdbHeader>::Err(
        Error(ErrorCode::kDataMissing, std::string(why) + "; run bf build to regenerate"));
  };
  if (std::memcmp(buf.data(), kMagic, 4) != 0) {
    return bad("not a .bfdb file (bad magic)");
  }
  ByteReader r(buf.data() + 4, buf.size() - 4);
  if (r.U32() != kFormatVersion) {
    return bad("incompatible .bfdb format version");
  }

  BfdbHeader header;
  header.cycle = r.U32();
  header.build = r.U32();
  // Skip the four count fields (v, e, airway_count, msa_count) and
  // first_airport_vertex; ReadHeader never touches the body they describe.
  for (int i = 0; i < 5; ++i) {
    r.U32();
  }
  auto read_inline = [&](std::string& s) {
    const uint32_t len = r.U32();
    if (!r.ok() || len > r.remaining()) {
      return false;
    }
    s.resize(len);
    for (uint32_t i = 0; i < len; ++i) {
      s[i] = static_cast<char>(r.U8());
    }
    return true;
  };
  if (!read_inline(header.program_semver) || !read_inline(header.source_loader)) {
    return bad("corrupt .bfdb header");
  }
  return Result<BfdbHeader>::Ok(std::move(header));
}

Result<BfdbImage> BfdbCache::Read(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f.is_open()) {
    return Result<BfdbImage>::Err(Error(ErrorCode::kDataMissing, "cannot open .bfdb: " + path));
  }
  const std::streamsize size = f.tellg();
  if (size < 4) {
    return Result<BfdbImage>::Err(Error(ErrorCode::kDataMissing, "truncated .bfdb: " + path));
  }
  std::string buf(static_cast<size_t>(size), '\0');
  f.seekg(0);
  f.read(buf.data(), size);
  if (!f) {
    return Result<BfdbImage>::Err(Error(ErrorCode::kDataMissing, "failed reading .bfdb: " + path));
  }

  auto bad = [&](const char* why) {
    return Result<BfdbImage>::Err(
        Error(ErrorCode::kDataMissing, std::string(why) + "; run bf build to regenerate"));
  };

  if (std::memcmp(buf.data(), kMagic, 4) != 0) {
    return bad("not a .bfdb file (bad magic)");
  }
  ByteReader r(buf.data() + 4, buf.size() - 4);
  const uint32_t format = r.U32();
  if (format != kFormatVersion) {
    return bad("incompatible .bfdb format version");
  }

  BfdbImage img;
  img.cycle = r.U32();
  img.build = r.U32();
  const uint32_t v = r.U32();
  const uint32_t e = r.U32();
  const uint32_t airway_count = r.U32();
  const uint32_t msa_count = r.U32();
  img.first_airport_vertex = static_cast<int>(r.U32());

  // Sanity-check the header counts against the bytes actually present BEFORE any
  // resize, so a corrupt or forged header cannot trigger a huge allocation (and
  // a bad_alloc/length_error that would bypass Result). Each count must fit in
  // the remaining bytes at its minimum on-disk element size; this is a necessary
  // condition, not an exact one -- a fuse, not a full validator. On-disk sizes:
  // vertex record 34 B (coord 16 + ident refs 16 + flags 1 + kind 1), airport
  // record 4 B (I32 elevation), offsets 4 B, GraphEdge 15 B (4+4+2+2+2+1, tighter
  // than the 16 B in-memory struct), airway ref 8 B, MSA sector at least 28 B
  // (6xU32 refs + a U32 arc count).
  //
  // first_airport_vertex must lie in [0, v]; the airport record section then
  // holds (v - first_airport_vertex) elevations. An out-of-range value is
  // rejected here rather than clamped, so the airport count below is trustworthy.
  if (!r.ok() || img.first_airport_vertex < 0 ||
      static_cast<uint32_t>(img.first_airport_vertex) > v) {
    return bad("corrupt .bfdb: first_airport_vertex out of range");
  }
  const uint32_t airport_count = v - static_cast<uint32_t>(img.first_airport_vertex);
  const size_t avail = r.remaining();
  auto count_fits = [&](uint32_t count, size_t per_elem) {
    return static_cast<size_t>(count) <= avail / per_elem;
  };
  if (!count_fits(v, 34) || !count_fits(airport_count, 4) || !count_fits(e, 15) ||
      !count_fits(airway_count, 8) || !count_fits(msa_count, 28)) {
    return bad("corrupt .bfdb: header counts exceed file size");
  }
  // Inline header strings (length-prefixed), in write order.
  auto read_inline = [&](std::string& s) {
    const uint32_t len = r.U32();
    if (!r.ok() || len > r.remaining()) {
      s.clear();
      return false;
    }
    s.resize(len);
    for (uint32_t i = 0; i < len; ++i) {
      s[i] = static_cast<char>(r.U8());
    }
    return true;
  };
  if (!read_inline(img.program_semver) || !read_inline(img.source_loader) ||
      !read_inline(img.data_dir)) {
    return bad("corrupt .bfdb header");
  }
  const uint32_t pool_len = r.U32();

  // Vertex records: coord + ident refs + flags + kind, one per vertex. Ident
  // refs are resolved against the string pool after it is read (below).
  struct IdentRef {
    uint32_t io, il, ro, rl;
  };
  img.coords.resize(v);
  img.on_network.assign(v, false);
  img.kinds.resize(v);
  std::vector<IdentRef> ident_refs(v);
  for (uint32_t i = 0; i < v; ++i) {
    img.coords[i].latitude = r.F64();
    img.coords[i].longitude = r.F64();
    ident_refs[i].io = r.U32();
    ident_refs[i].il = r.U32();
    ident_refs[i].ro = r.U32();
    ident_refs[i].rl = r.U32();
    const uint8_t flags = r.U8();
    img.on_network[i] = (flags & 0x01) != 0;
    img.kinds[i] = static_cast<WaypointKind>(r.U8());
  }
  // Airport records: elevation per airport vertex, in vertex order.
  // airport_count was validated against the file size in the header fuse above.
  img.airport_elevations_ft.resize(airport_count);
  for (uint32_t i = 0; i < airport_count; ++i) {
    img.airport_elevations_ft[i] = r.I32();
  }
  // offsets
  img.offsets.resize(static_cast<size_t>(v) + 1);
  for (uint32_t i = 0; i <= v; ++i) {
    img.offsets[i] = r.I32();
  }
  // edges
  img.edges.resize(e);
  for (uint32_t i = 0; i < e; ++i) {
    GraphEdge ed;
    ed.to = r.I32();
    ed.distance_nm = r.F32();
    ed.airway_id = r.U16();
    ed.base_fl = r.I16();
    ed.top_fl = r.I16();
    ed.flags = r.U8();
    img.edges[i] = ed;
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
  for (int16_t& c : cells) {
    c = r.I16();
  }
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
    if (!r.ok() || arc_count > r.remaining() / 12) {
      return bad("corrupt .bfdb msa section");
    }
    m.arcs.resize(arc_count);
    for (uint32_t a = 0; a < arc_count; ++a) {
      m.arcs[a].bearing_from = r.I32();
      m.arcs[a].alt_100ft = r.I32();
      m.arcs[a].radius_nm = r.I32();
    }
  }

  // String pool blob is the final section.
  if (!r.ok() || r.remaining() < pool_len) {
    return bad("corrupt .bfdb: truncated string pool");
  }
  const std::string blob = buf.substr(buf.size() - pool_len);

  // Resolve all references against the pool.
  bool refs_ok = true;
  img.idents.resize(v);
  for (uint32_t i = 0; i < v; ++i) {
    const IdentRef& ir = ident_refs[i];
    img.idents[i].ident = ResolveRef(blob, ir.io, ir.il, refs_ok);
    img.idents[i].region = ResolveRef(blob, ir.ro, ir.rl, refs_ok);
  }
  img.airway_names.resize(airway_count);
  for (uint32_t i = 0; i < airway_count; ++i) {
    img.airway_names[i] = ResolveRef(blob, airway_refs[i].o, airway_refs[i].l, refs_ok);
  }
  img.mora = MoraGrid::FromCells(std::move(cells));
  img.msa.resize(msa_count);
  for (uint32_t i = 0; i < msa_count; ++i) {
    MsaRef& m = msa_refs[i];
    img.msa[i].center.ident = ResolveRef(blob, m.cio, m.cil, refs_ok);
    img.msa[i].center.region = ResolveRef(blob, m.cro, m.crl, refs_ok);
    img.msa[i].airport_icao = ResolveRef(blob, m.aio, m.ail, refs_ok);
    img.msa[i].arcs = std::move(m.arcs);
  }

  if (!r.ok() || !refs_ok) {
    return bad("corrupt .bfdb: field or reference out of range");
  }
  return Result<BfdbImage>::Ok(std::move(img));
}

}  // namespace bf
