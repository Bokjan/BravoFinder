// SPDX-License-Identifier: MIT
// mcp_dispatch_test.cc — unit coverage of the transport-neutral MCP Dispatcher.
//
// The Dispatcher is the protocol core shared by the stdio and HTTP transports:
// it takes a parsed JSON-RPC request and returns the response envelope string.
// This exercise feeds it hand-built requests and asserts the returned envelope,
// covering the three MCP methods, the server-provided list_cycles tool, the
// JSON-RPC error codes (-32600 / -32601 / -32602), a notification (no response),
// cycle validation, and an unknown tool. It builds a registry over a minimal
// hand-constructed graph-only cache, so it needs no real navigation data and
// never SKIPs.

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <string>

#include "dispatcher.h"
#include "io/cache/bfdb_naming.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/unified_cache.h"
#include "jsonrpc.h"
#include "rapidjson/document.h"
#include "registry.h"

namespace {

namespace fs = std::filesystem;

// A minimal valid graph-only unified cache carrying an AIRAC cycle. Enough for
// the registry to open a (trivial) NavDatabase; the dispatch tests only need it
// to resolve, not to hold real navdata.
void WriteCache(const fs::path& dir, uint32_t cycle) {
  bf::GraphSnapshot g;
  g.first_airport_vertex = 0;
  g.offsets = {0};
  bf::UnifiedCache::BuildInput in;
  in.graph = &g;
  in.header.cycle = cycle;
  in.header.program_version = "test";
  in.header.source_loader = "test";
  const std::string path = (dir / bf::FormatBfdbName(cycle)).string();
  REQUIRE(bf::UnifiedCache::Build(path, in));
}

fs::path TempDir(const std::string& tag) {
  const fs::path dir = fs::temp_directory_path() / ("bravofinder_dispatch_" + tag);
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

bf::service::NavDatabaseRegistry MakeRegistry(const fs::path& dir) {
  bf::Result<bf::BfdbInventory> inv = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inv);
  return bf::service::NavDatabaseRegistry(std::move(inv.value()));
}

// Parse a Dispatcher response body back into a Document for structured asserts.
rapidjson::Document Parse(const std::string& body) {
  rapidjson::Document doc;
  doc.Parse(body.c_str());
  REQUIRE_FALSE(doc.HasParseError());
  return doc;
}

// Build a request Document from a JSON string.
rapidjson::Document Req(const char* json) {
  rapidjson::Document doc;
  doc.Parse(json);
  REQUIRE_FALSE(doc.HasParseError());
  return doc;
}

}  // namespace

TEST_CASE("mcp dispatcher: initialize negotiates the protocol version", "[unit][mcp]") {
  const fs::path dir = TempDir("init");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  SECTION("a client asking for 2025-03-26 gets it back") {
    const rapidjson::Document req = Req(
        R"({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-03-26"}})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["id"].GetInt() == 1);
    CHECK(std::string(doc["result"]["protocolVersion"].GetString()) == "2025-03-26");
    CHECK(std::string(doc["result"]["serverInfo"]["name"].GetString()) == "bf-mcp");
    CHECK(doc["result"]["capabilities"]["tools"].IsObject());
  }

  SECTION("a client sending no protocolVersion falls back to the newest supported version") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":2,"method":"initialize"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(std::string(doc["result"]["protocolVersion"].GetString()) == "2025-03-26");
  }

  SECTION("an unsupported protocolVersion falls back to the newest supported version") {
    const rapidjson::Document req = Req(
        R"({"jsonrpc":"2.0","id":3,"method":"initialize","params":{"protocolVersion":"1999-01-01"}})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(std::string(doc["result"]["protocolVersion"].GetString()) == "2025-03-26");
  }
}

TEST_CASE("mcp dispatcher: tools/list enumerates the tools plus list_cycles", "[unit][mcp]") {
  const fs::path dir = TempDir("list");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":1,"method":"tools/list"})");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  REQUIRE(doc["result"]["tools"].IsArray());
  const rapidjson::Value& tools = doc["result"]["tools"];
  // The nine shared tools plus the server-provided list_cycles.
  CHECK(tools.Size() == 10);
  bool saw_find_routes = false;
  bool saw_list_cycles = false;
  bool find_routes_has_cycle = false;
  for (const rapidjson::Value& t : tools.GetArray()) {
    const std::string name = t["name"].GetString();
    if (name == "find_routes") {
      saw_find_routes = true;
      // The shared "cycle" property is injected into every per-database tool.
      find_routes_has_cycle = t["inputSchema"]["properties"].HasMember("cycle");
    }
    if (name == "list_cycles") {
      saw_list_cycles = true;
    }
  }
  CHECK(saw_find_routes);
  CHECK(saw_list_cycles);
  CHECK(find_routes_has_cycle);
}

TEST_CASE("mcp dispatcher: tools/call list_cycles returns the available cycles", "[unit][mcp]") {
  const fs::path dir = TempDir("cycles");
  WriteCache(dir, 2601);
  WriteCache(dir, 2602);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req =
      Req(R"({"jsonrpc":"2.0","id":7,"method":"tools/call","params":{"name":"list_cycles"}})");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  CHECK_FALSE(doc["result"]["isError"].GetBool());
  // The tool text is a serialized JSON array of {cycle}; parse it and check both
  // cycles are present, newest first.
  rapidjson::Document inner;
  inner.Parse(doc["result"]["content"][0]["text"].GetString());
  REQUIRE_FALSE(inner.HasParseError());
  REQUIRE(inner.IsArray());
  REQUIRE(inner.Size() == 2);
  CHECK(inner[0]["cycle"].GetUint() == 2602);
  CHECK(inner[1]["cycle"].GetUint() == 2601);
}

TEST_CASE("mcp dispatcher: tools/call rejects a non-integer cycle", "[unit][mcp]") {
  const fs::path dir = TempDir("badcycle");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req = Req(
      R"({"jsonrpc":"2.0","id":8,"method":"tools/call","params":{"name":"lookup_airports","arguments":{"ids":["KJFK"],"cycle":-1}}})");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  CHECK(doc["result"]["isError"].GetBool());
}

TEST_CASE("mcp dispatcher: tools/call on an unknown tool is a tool error", "[unit][mcp]") {
  const fs::path dir = TempDir("unknown");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req = Req(
      R"({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{"name":"no_such_tool","arguments":{}}})");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  CHECK(doc["result"]["isError"].GetBool());
  CHECK(std::string(doc["result"]["content"][0]["text"].GetString()).find("unknown tool") !=
        std::string::npos);
}

TEST_CASE("mcp dispatcher: JSON-RPC protocol errors", "[unit][mcp]") {
  bf::service::NavDatabaseRegistry reg(bf::BfdbInventory{});  // no cycles needed
  bf::mcp::Dispatcher dispatcher(reg);

  SECTION("missing method with an id is -32600") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":1})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
  }

  SECTION("an unknown method with an id is -32601") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":2,"method":"no/such"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kMethodNotFound);
  }

  SECTION("tools/call without a params object is -32602") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":3,"method":"tools/call"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidParams);
  }

  SECTION("missing jsonrpc is -32600") {
    const rapidjson::Document req = Req(R"({"id":4,"method":"tools/list"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
    CHECK(doc["id"].GetInt() == 4);
  }

  SECTION("wrong jsonrpc version is -32600") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"1.0","id":5,"method":"tools/list"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
  }

  SECTION("missing jsonrpc without an id still replies with id null") {
    const rapidjson::Document req = Req(R"({"method":"initialize"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
    CHECK(doc["id"].IsNull());
  }
}

TEST_CASE("mcp dispatcher: a notification yields no response", "[unit][mcp]") {
  bf::service::NavDatabaseRegistry reg(bf::BfdbInventory{});
  bf::mcp::Dispatcher dispatcher(reg);

  SECTION("an initialize notification (no id) is silent") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","method":"initialize"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    CHECK_FALSE(resp.has_response);
  }

  SECTION("a method-less notification is silent") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    CHECK_FALSE(resp.has_response);
  }
}

TEST_CASE("mcp dispatcher: a batch dispatches each element", "[unit][mcp]") {
  const fs::path dir = TempDir("batch");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  SECTION("a batch returns an array of the non-notification responses") {
    const rapidjson::Document req = Req(
        R"([{"jsonrpc":"2.0","id":1,"method":"tools/list"},{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"list_cycles"}}])");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    REQUIRE(doc.IsArray());
    CHECK(doc.Size() == 2);
  }

  SECTION("a non-object batch element becomes a -32600 entry, not a drop") {
    // A valid request, a non-object (42), and a notification: the array holds
    // the request's result and a -32600 for the 42, nothing for the notification.
    const rapidjson::Document req = Req(
        R"([{"jsonrpc":"2.0","id":1,"method":"tools/list"},42,{"jsonrpc":"2.0","method":"n"}])");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    REQUIRE(doc.IsArray());
    REQUIRE(doc.Size() == 2);
    CHECK(doc[0]["id"].GetInt() == 1);
    CHECK(doc[1]["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
  }

  SECTION("a nested array batch element is -32600, not a recursive batch") {
    const rapidjson::Document req = Req(
        R"([{"jsonrpc":"2.0","id":1,"method":"tools/list"},[{"jsonrpc":"2.0","id":2,"method":"tools/list"}]])");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    REQUIRE(doc.IsArray());
    REQUIRE(doc.Size() == 2);
    CHECK(doc[0]["id"].GetInt() == 1);
    CHECK(doc[1]["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
    // Nested request id 2 must not appear as a successful result entry.
    CHECK_FALSE(doc[1].HasMember("result"));
  }

  SECTION("a batch element missing jsonrpc is -32600") {
    const rapidjson::Document req =
        Req(R"([{"id":1,"method":"tools/list"},{"jsonrpc":"2.0","id":2,"method":"tools/list"}])");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    REQUIRE(doc.IsArray());
    REQUIRE(doc.Size() == 2);
    CHECK(doc[0]["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
    CHECK(doc[1]["id"].GetInt() == 2);
    CHECK(doc[1].HasMember("result"));
  }

  SECTION("a batch of only notifications yields no response") {
    const rapidjson::Document req =
        Req(R"([{"jsonrpc":"2.0","method":"a"},{"jsonrpc":"2.0","method":"b"}])");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    CHECK_FALSE(resp.has_response);
  }

  SECTION("an empty batch is a single -32600 error envelope, not an array") {
    const rapidjson::Document req = Req("[]");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    REQUIRE(doc.IsObject());
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
  }
}
