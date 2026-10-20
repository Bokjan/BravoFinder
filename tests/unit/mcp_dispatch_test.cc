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

// Build a request Document from a JSON string, parsed the way both transports
// parse a body: iteratively, so a deeply nested value cannot overflow the
// recursive parser's stack before the dispatcher even sees it.
rapidjson::Document Req(const char* json) {
  rapidjson::Document doc;
  doc.Parse<rapidjson::kParseIterativeFlag>(json);
  REQUIRE_FALSE(doc.HasParseError());
  return doc;
}

// Same, for a body that legitimately contains a NUL byte (the raw JSON text
// spelled "\u0000"), which the const char* overload would truncate at.
rapidjson::Document Req(const std::string& json) {
  rapidjson::Document doc;
  doc.Parse<rapidjson::kParseIterativeFlag>(json.data(), json.size());
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
  // The ten shared tools plus the server-provided list_cycles.
  CHECK(tools.Size() == 11);
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

TEST_CASE("mcp dispatcher: tools/call on an unknown tool is -32602", "[unit][mcp]") {
  const fs::path dir = TempDir("unknown");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req = Req(
      R"({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{"name":"no_such_tool","arguments":{}}})");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidParams);
  CHECK(doc["id"].GetInt() == 9);
  CHECK(std::string(doc["error"]["message"].GetString()).find("unknown tool") != std::string::npos);
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

TEST_CASE("mcp dispatcher: only a string or number id is echoed", "[unit][mcp]") {
  bf::service::NavDatabaseRegistry reg(bf::BfdbInventory{});
  bf::mcp::Dispatcher dispatcher(reg);

  // A deeply nested array id: echoing it would recurse once per level through
  // rapidjson's GenericValue::Accept and overflow the worker stack. It must be
  // answered with -32600 and a null id instead of being echoed.
  SECTION("a nested array id is refused, not echoed") {
    std::string deep;
    for (int i = 0; i < 100000; ++i) {
      deep.push_back('[');
    }
    for (int i = 0; i < 100000; ++i) {
      deep.push_back(']');
    }
    const rapidjson::Document req =
        Req((R"({"jsonrpc":"2.0","method":"tools/list","id":)" + deep + "}").c_str());
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
    CHECK(doc["id"].IsNull());
  }

  SECTION("an object, boolean or null id is -32600 with a null id") {
    for (const char* json : {R"({"jsonrpc":"2.0","method":"tools/list","id":{"a":1}})",
                             R"({"jsonrpc":"2.0","method":"tools/list","id":[1,2]})",
                             R"({"jsonrpc":"2.0","method":"tools/list","id":true})",
                             R"({"jsonrpc":"2.0","method":"tools/list","id":null})"}) {
      const rapidjson::Document req = Req(json);
      const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
      REQUIRE(resp.has_response);
      rapidjson::Document doc = Parse(resp.body);
      CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
      CHECK(doc["id"].IsNull());
    }
  }

  SECTION("a string or integer id round-trips unchanged") {
    const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":"req-7","method":"ping"})");
    const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
    REQUIRE(resp.has_response);
    rapidjson::Document doc = Parse(resp.body);
    CHECK(std::string(doc["id"].GetString()) == "req-7");
  }
}

TEST_CASE("mcp dispatcher: ping returns an empty result", "[unit][mcp]") {
  bf::service::NavDatabaseRegistry reg(bf::BfdbInventory{});
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req = Req(R"({"jsonrpc":"2.0","id":11,"method":"ping"})");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  REQUIRE(doc.HasMember("result"));
  CHECK(doc["result"].IsObject());
  CHECK(doc["result"].MemberCount() == 0);
  CHECK(doc["id"].GetInt() == 11);

  // A ping notification is silent like any other notification.
  const rapidjson::Document note = Req(R"({"jsonrpc":"2.0","method":"ping"})");
  CHECK_FALSE(dispatcher.Dispatch(note).has_response);
}

TEST_CASE("mcp dispatcher: a NUL inside a method or tool name does not truncate", "[unit][mcp]") {
  const fs::path dir = TempDir("nul");
  WriteCache(dir, 2601);
  bf::service::NavDatabaseRegistry reg = MakeRegistry(dir);
  bf::mcp::Dispatcher dispatcher(reg);

  // The body spells the escape "\u0000" as raw JSON text, so the parsed method
  // literally contains a NUL. It is not "ping": the full string length decides.
  const std::string ping =
      std::string(R"({"jsonrpc":"2.0","id":1,"method":"ping)") + "\u0000" + R"(junk"})";
  const rapidjson::Document req = Req(ping);
  // "ping", the NUL, "junk" (8 bytes): the length includes the NUL, so a bare
  // GetString() comparison would only ever see "ping".
  REQUIRE(req["method"].GetStringLength() == 8);
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kMethodNotFound);

  // Same for a tool name: "list_cycles\u0000junk" resolves to no such tool.
  const std::string call =
      std::string(
          R"({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"list_cycles)") +
      "\u0000" + R"(junk"}})";
  const rapidjson::Document call_req = Req(call);
  const bf::mcp::Dispatcher::Response call_resp = dispatcher.Dispatch(call_req);
  REQUIRE(call_resp.has_response);
  rapidjson::Document call_doc = Parse(call_resp.body);
  CHECK(call_doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidParams);
}

TEST_CASE("mcp dispatcher: initialize is refused inside a batch", "[unit][mcp]") {
  bf::service::NavDatabaseRegistry reg(bf::BfdbInventory{});
  bf::mcp::Dispatcher dispatcher(reg);

  const rapidjson::Document req = Req(
      R"([{"jsonrpc":"2.0","id":1,"method":"initialize"},{"jsonrpc":"2.0","id":2,"method":"ping"}])");
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  REQUIRE(doc.IsArray());
  REQUIRE(doc.Size() == 2);
  CHECK(doc[0]["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
  CHECK(doc[0]["id"].GetInt() == 1);
  CHECK(doc[1]["result"].IsObject());
}

TEST_CASE("mcp dispatcher: a batch larger than the cap is refused whole", "[unit][mcp]") {
  bf::service::NavDatabaseRegistry reg(bf::BfdbInventory{});
  bf::mcp::Dispatcher dispatcher(reg);

  std::string json = "[";
  for (std::size_t i = 0; i <= bf::mcp::kMaxBatchSize; ++i) {
    if (i > 0) {
      json += ",";
    }
    json += R"({"jsonrpc":"2.0","id":)" + std::to_string(i) + R"(,"method":"ping"})";
  }
  json += "]";
  const rapidjson::Document req = Req(json.c_str());
  const bf::mcp::Dispatcher::Response resp = dispatcher.Dispatch(req);
  REQUIRE(resp.has_response);
  rapidjson::Document doc = Parse(resp.body);
  // One error envelope for the batch, not a per-element array.
  CHECK(doc.IsObject());
  CHECK(doc["error"]["code"].GetInt() == bf::mcp::jsonrpc::kInvalidRequest);
  CHECK(doc["id"].IsNull());

  // A batch exactly at the cap is still served.
  const rapidjson::Document at_cap = Req([] {
    std::string j = "[";
    for (std::size_t i = 0; i < bf::mcp::kMaxBatchSize; ++i) {
      if (i > 0) {
        j += ",";
      }
      j += R"({"jsonrpc":"2.0","id":)" + std::to_string(i) + R"(,"method":"ping"})";
    }
    return j + "]";
  }()
                                             .c_str());
  const bf::mcp::Dispatcher::Response at_cap_resp = dispatcher.Dispatch(at_cap);
  REQUIRE(at_cap_resp.has_response);
  rapidjson::Document at_cap_doc = Parse(at_cap_resp.body);
  REQUIRE(at_cap_doc.IsArray());
  CHECK(at_cap_doc.Size() == bf::mcp::kMaxBatchSize);
}
