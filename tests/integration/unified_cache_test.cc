#include "io/cache/unified_cache.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "io/cache/cifp_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"
#include "io/nav_data.h"

namespace {

// A unique temp path so parallel cases do not collide. Uses the platform temp
// dir so the test runs on Windows (where /tmp is absent).
std::string TempBfdb(const std::string& tag) {
  std::filesystem::path dir = std::filesystem::temp_directory_path();
  return (dir / ("bravofinder_unified_test_" + tag + ".bfdb")).string();
}

// A minimal but valid graph snapshot: two vertices (one waypoint, one airport),
// one edge, distinct idents so the shared pool has something to dedupe.
bf::GraphSnapshot MakeGraph() {
  bf::GraphSnapshot g;
  g.first_airport_vertex = 1;  // vertex 0 waypoint, vertex 1 airport
  g.coords = {{40.0, -73.0}, {41.0, -74.0}};
  g.idents = {bf::FixedIdent::FromParts("WAYPT", "K6"), bf::FixedIdent::FromParts("KTST", "K6")};
  g.kinds = {bf::WaypointKind::kFix, bf::WaypointKind::kOther};
  g.has_outbound = {1, 0};  // vertex 0 has an out-edge, vertex 1 has none
  g.has_inbound = {0, 1};   // vertex 1 is the edge's destination, vertex 0 is not
  g.offsets = {0, 1, 1};    // vertex 0 has one out-edge, vertex 1 has none
  g.edges = {bf::GraphEdge{}};
  g.edges[0].to = 1;
  g.edges[0].distance_nm = 42.0f;
  g.airway_names = {"J1"};
  g.airport_elevations_ft = {13};  // one airport
  return g;
}

// A CifpData with one procedure and one runway, its strings drawn to overlap
// with the graph's idents so the global pool dedupes across sections.
bf::CifpData MakeCifp() {
  bf::CifpData d{};
  bf::ProcedureLeg leg{};
  leg.fix = bf::FixedIdent::FromParts("WAYPT", "K6");  // same string as a graph ident
  leg.path_term = bf::PathTerminator::kTF;
  bf::Procedure p{
      .type = bf::ProcedureType::kSid,
      .name = "TESTSID",
      .runway = "04L",
      .legs = {leg},
  };
  d.procedures.push_back(std::move(p));
  bf::Runway rwy;
  rwy.ident = "04L";
  rwy.threshold = {40.5, -73.5};
  rwy.elevation_ft = 13;
  d.runways.push_back(rwy);
  return d;
}

bf::NavDetailArchive MakeDetail() {
  bf::NavData data;
  bf::NavaidDetail nav;
  nav.ident = bf::Ident("WAYPT", "K6");  // shared string again
  nav.kind = bf::WaypointKind::kVor;
  nav.elev_ft = 100;
  nav.freq_raw = 11350;
  nav.range_nm = 130.0;
  data.navaid_details.push_back(nav);
  return bf::NavDetailArchive::FromData(data);
}

}  // namespace

TEST_CASE("unified: a full round-trip preserves all three sections", "[integration][unified]") {
  const std::string path = TempBfdb("full");
  const bf::GraphSnapshot graph = MakeGraph();
  const std::vector<std::pair<std::string, bf::CifpData>> cifp = {{"KTST", MakeCifp()}};
  const bf::NavDetailArchive detail = MakeDetail();

  bf::UnifiedCache::BuildInput in;
  in.graph = &graph;
  in.cifp = &cifp;
  in.detail = &detail;
  in.header.cycle = 2601;
  in.header.program_semver = "3.3.0";
  in.header.source_loader = "xplane12";
  in.header.data_dir = "/data/navdata";
  REQUIRE(bf::UnifiedCache::Build(path, in));

  bf::Result<bf::UnifiedData> opened = bf::UnifiedCache::Open(path);
  REQUIRE(opened);
  const bf::UnifiedData& u = opened.value();

  // Header
  CHECK(u.header.cycle == 2601);
  CHECK(u.header.program_semver == "3.3.0");
  CHECK(u.header.source_loader == "xplane12");
  CHECK(u.header.data_dir == "/data/navdata");

  // Graph section
  REQUIRE(u.graph.coords.size() == 2);
  CHECK(u.graph.first_airport_vertex == 1);
  CHECK(u.graph.idents[0].IdentView() == "WAYPT");
  CHECK(u.graph.idents[1].IdentView() == "KTST");
  REQUIRE(u.graph.edges.size() == 1);
  CHECK(u.graph.edges[0].to == 1);
  REQUIRE(u.graph.airway_names.size() == 1);
  CHECK(u.graph.airway_names[0] == "J1");
  REQUIRE(u.graph.airport_elevations_ft.size() == 1);
  CHECK(u.graph.airport_elevations_ft[0] == 13);

  // CIFP section: on-demand fetch of the one airport.
  REQUIRE(u.cifp.has_value());
  CHECK(u.cifp->Has("KTST"));
  auto fetched = u.cifp->Fetch("KTST");
  REQUIRE(fetched.has_value());
  REQUIRE(fetched->procedures.size() == 1);
  CHECK(fetched->procedures[0].name == "TESTSID");
  REQUIRE(fetched->procedures[0].legs.size() == 1);
  CHECK(fetched->procedures[0].legs[0].fix.IdentView() == "WAYPT");
  REQUIRE(fetched->runways.size() == 1);
  CHECK(fetched->runways[0].ident == "04L");

  // Detail section
  REQUIRE(u.detail.has_value());
  std::vector<bf::NavaidDetailInfo> navs = u.detail->FindNavaids("WAYPT");
  REQUIRE(navs.size() == 1);
  CHECK(navs[0].freq_raw == 11350);
  CHECK(navs[0].kind == bf::WaypointKind::kVor);

  std::remove(path.c_str());
}

// The has_inbound / has_outbound per-vertex flags (the v7 cache layout: flags
// byte bit0 = has_outbound, bit1 = has_inbound) must survive a round-trip, and
// specifically an inbound-only vertex (has_inbound && !has_outbound -- a STAR
// entry gate reached only via a forward-only airway) must not be collapsed to
// off-network. This is a data-independent invariant: MakeGraph's vertex 1 is
// inbound-only by construction, so unlike the real-data graph_codec_test case it
// holds across every AIRAC cycle rather than depending on the data happening to
// contain a forward-only dead end.
TEST_CASE("unified: inbound-only vertex flags survive the round-trip", "[integration][unified]") {
  const std::string path = TempBfdb("inboundonly");
  const bf::GraphSnapshot graph = MakeGraph();
  // Sanity: vertex 1 is inbound-only going in (the precondition this test pins).
  REQUIRE(graph.has_outbound.size() == 2);
  REQUIRE(graph.has_inbound.size() == 2);
  REQUIRE(graph.has_outbound[1] == 0);
  REQUIRE(graph.has_inbound[1] == 1);

  bf::UnifiedCache::BuildInput in;
  in.graph = &graph;
  in.header.cycle = 2603;
  REQUIRE(bf::UnifiedCache::Build(path, in));

  bf::Result<bf::UnifiedData> opened = bf::UnifiedCache::Open(path);
  REQUIRE(opened);
  const bf::GraphSnapshot& g = opened.value().graph;
  REQUIRE(g.has_outbound.size() == 2);
  REQUIRE(g.has_inbound.size() == 2);

  // bit0 (has_outbound) and bit1 (has_inbound) both round-trip exactly, vertex
  // for vertex -- no smearing of one flag onto the other.
  CHECK(g.has_outbound[0] == 1);
  CHECK(g.has_inbound[0] == 0);
  CHECK(g.has_outbound[1] == 0);
  CHECK(g.has_inbound[1] == 1);

  std::remove(path.c_str());
}

TEST_CASE("unified: ReadHeader returns provenance without decoding sections",
          "[integration][unified]") {
  const std::string path = TempBfdb("header");
  const bf::GraphSnapshot graph = MakeGraph();
  bf::UnifiedCache::BuildInput in;
  in.graph = &graph;
  in.header.cycle = 2603;
  in.header.source_loader = "xplane12";
  REQUIRE(bf::UnifiedCache::Build(path, in));

  bf::Result<bf::UnifiedHeader> h = bf::UnifiedCache::ReadHeader(path);
  REQUIRE(h);
  CHECK(h.value().cycle == 2603);
  CHECK(h.value().source_loader == "xplane12");

  std::remove(path.c_str());
}

TEST_CASE("unified: a corrupt or missing file is rejected cleanly", "[unit][unified]") {
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
}
