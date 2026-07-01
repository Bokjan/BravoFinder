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
  if (image.offsets.size() != v + 1 || image.idents.size() != v || image.on_network.size() != v) {
    return Result<void>::Err(Error(ErrorCode::kParseError, "inconsistent image array sizes"));
  }

  // Sections are serialized into their own buffers first; string references are
  // accumulated into a shared pool, whose blob is appended last.
  StringPool pool;
  std::string sections;
  ByteWriter w(sections);

  // coords: V * (lat, lon)
  for (const Coordinate& c : image.coords) {
    w.F64(c.latitude);
    w.F64(c.longitude);
  }
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
  // on_network: bit-packed, ceil(V / 8) bytes
  for (size_t i = 0; i < v; i += 8) {
    uint8_t byte = 0;
    for (size_t b = 0; b < 8 && i + b < v; ++b) {
      if (image.on_network[i + b]) {
        byte |= static_cast<uint8_t>(1u << b);
      }
    }
    w.U8(byte);
  }
  // idents: V * (ident_ref, region_ref)
  for (const Ident& id : image.idents) {
    const auto ir = pool.Add(id.ident);
    const auto rr = pool.Add(id.region);
    w.U32(ir.first);
    w.U32(ir.second);
    w.U32(rr.first);
    w.U32(rr.second);
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

  // coords
  img.coords.resize(v);
  for (uint32_t i = 0; i < v; ++i) {
    img.coords[i].latitude = r.F64();
    img.coords[i].longitude = r.F64();
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
  // on_network: bit-packed
  img.on_network.assign(v, false);
  for (uint32_t i = 0; i < v; i += 8) {
    const uint8_t byte = r.U8();
    for (uint32_t b = 0; b < 8 && i + b < v; ++b) {
      if (byte & (1u << b)) {
        img.on_network[i + b] = true;
      }
    }
  }
  // idents (references into the pool, resolved after the pool is read)
  struct IdentRef {
    uint32_t io, il, ro, rl;
  };
  std::vector<IdentRef> ident_refs(v);
  for (uint32_t i = 0; i < v; ++i) {
    ident_refs[i].io = r.U32();
    ident_refs[i].il = r.U32();
    ident_refs[i].ro = r.U32();
    ident_refs[i].rl = r.U32();
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
