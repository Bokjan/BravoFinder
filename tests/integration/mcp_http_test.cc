// mcp_http_test.cc — end-to-end coverage of the MCP-over-HTTP (Streamable HTTP)
// transport: a real McpHttpHandler on a libuv server bound to a loopback port,
// driven over an actual socket. It exercises the Streamable HTTP surface:
// initialize (session id + protocol negotiation), tools/list, a tools/call, a
// notification (202), a batch, application/json vs text/event-stream content
// negotiation, the GET SSE stream, DELETE, an unknown session id (accepted), and
// the framing-error paths. It builds a registry over a minimal hand-constructed
// graph-only cache, so it needs no real navigation data and never SKIPs.
//
// The client uses POSIX sockets, so the whole suite is skipped on Windows.

#include <catch2/catch_test_macros.hpp>

#if defined(_WIN32)

TEST_CASE("mcp-over-http e2e: skipped on Windows", "[integration][mcp]") {
  SKIP("MCP-over-HTTP end-to-end test uses POSIX sockets; not run on Windows");
}

#else

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#include <uv.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <future>
#include <string>
#include <thread>

#include "io/cache/bfdb_inventory.h"
#include "io/cache/bfdb_naming.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/unified_cache.h"
#include "mcp_http.h"
#include "rapidjson/document.h"
#include "registry.h"
#include "server.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace {

namespace fs = std::filesystem;

// A minimal valid graph-only unified cache carrying an AIRAC cycle -- enough for
// the registry to open a (trivial) NavDatabase. The transport tests only need it
// to resolve; they do not depend on real navdata.
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
  const fs::path dir = fs::temp_directory_path() / ("bravofinder_mcphttp_" + tag);
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

// An McpHttpHandler server on its own loop thread, bound to a loopback port.
// Mirrors http_test.cc's ServerHarness, but wires McpHttpHandler as the
// RequestHandler instead of the REST Router.
class McpHarness {
 public:
  McpHarness(bf::service::NavDatabaseRegistry& registry, const bf::http_server::Limits& limits)
      : registry_(registry), limits_(limits) {
    std::promise<int> port_promise;
    std::future<int> port_future = port_promise.get_future();
    thread_ = std::thread([this, p = std::move(port_promise)]() mutable { RunLoop(std::move(p)); });
    port_ = port_future.get();
  }

  ~McpHarness() {
    uv_async_send(&stop_);
    thread_.join();
  }

  int port() const { return port_; }

 private:
  void RunLoop(std::promise<int> port_promise) {
    uv_loop_init(&loop_);
    uv_async_init(&loop_, &stop_, [](uv_async_t* a) {
      auto* server = static_cast<bf::http_server::Server*>(a->data);
      server->Close();
      uv_close(reinterpret_cast<uv_handle_t*>(a), nullptr);
    });
    bf::mcp::McpHttpHandler handler(registry_, &loop_);
    bf::http_server::Server server(&loop_, handler, limits_);
    stop_.data = &server;
    const int r = server.Listen("127.0.0.1", 0);
    port_promise.set_value(r == 0 ? server.BoundPort() : -1);
    uv_run(&loop_, UV_RUN_DEFAULT);
    uv_walk(
        &loop_,
        [](uv_handle_t* h, void*) {
          if (uv_is_closing(h) == 0) {
            uv_close(h, nullptr);
          }
        },
        nullptr);
    uv_run(&loop_, UV_RUN_DEFAULT);
    uv_loop_close(&loop_);
  }

  bf::service::NavDatabaseRegistry& registry_;
  bf::http_server::Limits limits_;
  uv_loop_t loop_{};
  uv_async_t stop_{};
  std::thread thread_;
  int port_ = -1;
};

int ConnectTo(int port, int recv_timeout_sec) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  timeval tv{recv_timeout_sec, 0};
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  if (connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return -1;
  }
  return fd;
}

bool SendAll(int fd, const std::string& data) {
  size_t off = 0;
  while (off < data.size()) {
    const ssize_t n = send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

size_t ParseContentLength(const std::string& headers) {
  const std::string key = "Content-Length: ";
  const size_t pos = headers.find(key);
  if (pos == std::string::npos) {
    return 0;
  }
  return static_cast<size_t>(std::stoul(headers.substr(pos + key.size())));
}

// Read one full Content-Length-framed HTTP response.
std::string ReadResponse(int fd) {
  std::string buf;
  char tmp[4096];
  bool have_headers = false;
  size_t header_end = std::string::npos;
  size_t content_len = 0;
  while (true) {
    if (have_headers && buf.size() >= header_end + 4 + content_len) {
      break;
    }
    const ssize_t n = recv(fd, tmp, sizeof(tmp), 0);
    if (n <= 0) {
      break;
    }
    buf.append(tmp, static_cast<size_t>(n));
    if (!have_headers) {
      header_end = buf.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        have_headers = true;
        content_len = ParseContentLength(buf.substr(0, header_end));
      }
    }
  }
  return buf;
}

// Read whatever arrives (for an open, unframed SSE stream): recv until the
// client-side timeout or EOF, capped so a still-open stream cannot block forever.
std::string ReadAvailable(int fd) {
  std::string buf;
  char tmp[4096];
  while (buf.size() < 8192) {
    const ssize_t n = recv(fd, tmp, sizeof(tmp), 0);
    if (n <= 0) {
      break;  // timeout or EOF
    }
    buf.append(tmp, static_cast<size_t>(n));
  }
  return buf;
}

int StatusOf(const std::string& response) {
  const std::string prefix = "HTTP/1.1 ";
  if (response.rfind(prefix, 0) != 0) {
    return -1;
  }
  return std::stoi(response.substr(prefix.size(), 3));
}

std::string HeaderBlock(const std::string& response) {
  const size_t end = response.find("\r\n\r\n");
  return response.substr(0, end == std::string::npos ? response.size() : end);
}

std::string BodyOf(const std::string& response) {
  const size_t end = response.find("\r\n\r\n");
  return end == std::string::npos ? "" : response.substr(end + 4);
}

bool HasHeader(const std::string& response, const std::string& needle) {
  return HeaderBlock(response).find(needle) != std::string::npos;
}

std::string PostMcp(const std::string& body, const std::string& accept,
                    const std::string& extra = "") {
  std::string r = "POST /mcp HTTP/1.1\r\nHost: x\r\nConnection: close\r\n";
  if (!accept.empty()) {
    r += "Accept: " + accept + "\r\n";
  }
  r += extra;
  r += "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
  r += body;
  return r;
}

// One request over a fresh connection; returns the full framed response.
std::string RoundTrip(int port, const std::string& request) {
  const int fd = ConnectTo(port, 5);
  if (fd < 0) {
    return "";
  }
  std::string response;
  if (SendAll(fd, request)) {
    response = ReadResponse(fd);
  }
  close(fd);
  return response;
}

// Parse a response body as JSON (application/json path).
rapidjson::Document ParseBody(const std::string& response) {
  rapidjson::Document doc;
  doc.Parse(BodyOf(response).c_str());
  return doc;
}

}  // namespace

TEST_CASE("mcp-over-http end-to-end over a loopback socket", "[integration][mcp]") {
  signal(SIGPIPE, SIG_IGN);

  const fs::path dir = TempDir("e2e");
  WriteCache(dir, 2601);
  WriteCache(dir, 2602);
  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(dir.string());
  REQUIRE(inventory);
  bf::service::NavDatabaseRegistry registry(std::move(inventory.value()));

  bf::http_server::Limits limits;
  McpHarness harness(registry, limits);
  REQUIRE(harness.port() > 0);
  const int port = harness.port();

  SECTION("initialize negotiates 2025-03-26 and returns a session id") {
    const std::string resp = RoundTrip(
        port,
        PostMcp(
            R"({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-03-26"}})",
            "application/json"));
    REQUIRE(StatusOf(resp) == 200);
    CHECK(HasHeader(resp, "Content-Type: application/json"));
    CHECK(HasHeader(resp, "Mcp-Session-Id: "));
    rapidjson::Document doc = ParseBody(resp);
    REQUIRE_FALSE(doc.HasParseError());
    CHECK(doc["id"].GetInt() == 1);
    CHECK(std::string(doc["result"]["protocolVersion"].GetString()) == "2025-03-26");
    CHECK(std::string(doc["result"]["serverInfo"]["name"].GetString()) == "bf-mcp");
  }

  SECTION("tools/list returns the tool set") {
    const std::string resp = RoundTrip(
        port, PostMcp(R"({"jsonrpc":"2.0","id":2,"method":"tools/list"})", "application/json"));
    REQUIRE(StatusOf(resp) == 200);
    rapidjson::Document doc = ParseBody(resp);
    REQUIRE(doc["result"]["tools"].IsArray());
    CHECK(doc["result"]["tools"].Size() == 10);
  }

  SECTION("tools/call list_cycles returns the available cycles") {
    const std::string resp = RoundTrip(
        port,
        PostMcp(R"({"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"list_cycles"}})",
                "application/json"));
    REQUIRE(StatusOf(resp) == 200);
    rapidjson::Document doc = ParseBody(resp);
    CHECK_FALSE(doc["result"]["isError"].GetBool());
    rapidjson::Document inner;
    inner.Parse(doc["result"]["content"][0]["text"].GetString());
    REQUIRE(inner.IsArray());
    CHECK(inner.Size() == 2);
    CHECK(inner[0]["cycle"].GetUint() == 2602);
  }

  SECTION("a notification is acknowledged with 202 and no body") {
    const std::string resp = RoundTrip(
        port,
        PostMcp(R"({"jsonrpc":"2.0","method":"notifications/initialized"})", "application/json"));
    CHECK(StatusOf(resp) == 202);
    CHECK(BodyOf(resp).empty());
  }

  SECTION("a batch returns an array of the non-notification responses") {
    const std::string resp = RoundTrip(
        port,
        PostMcp(
            R"([{"jsonrpc":"2.0","id":1,"method":"tools/list"},{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"list_cycles"}}])",
            "application/json"));
    REQUIRE(StatusOf(resp) == 200);
    rapidjson::Document doc = ParseBody(resp);
    REQUIRE(doc.IsArray());
    CHECK(doc.Size() == 2);
  }

  SECTION("a batch of only notifications is 202") {
    const std::string resp = RoundTrip(
        port, PostMcp(R"([{"jsonrpc":"2.0","method":"a"},{"jsonrpc":"2.0","method":"b"}])",
                      "application/json"));
    CHECK(StatusOf(resp) == 202);
  }

  SECTION("Accept: text/event-stream yields a single SSE event") {
    const std::string resp = RoundTrip(
        port, PostMcp(R"({"jsonrpc":"2.0","id":9,"method":"tools/list"})", "text/event-stream"));
    REQUIRE(StatusOf(resp) == 200);
    CHECK(HasHeader(resp, "Content-Type: text/event-stream"));
    const std::string body = BodyOf(resp);
    CHECK(body.rfind("data: ", 0) == 0);  // starts with the SSE data prefix
    CHECK(body.find("\"tools\"") != std::string::npos);
  }

  SECTION("an unknown Mcp-Session-Id is accepted, not rejected") {
    const std::string resp =
        RoundTrip(port, PostMcp(R"({"jsonrpc":"2.0","id":4,"method":"tools/list"})",
                                "application/json", "Mcp-Session-Id: deadbeefdeadbeef\r\n"));
    CHECK(StatusOf(resp) == 200);
  }

  SECTION("DELETE /mcp is acknowledged with 200") {
    std::string req = "DELETE /mcp HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n";
    CHECK(StatusOf(RoundTrip(port, req)) == 200);
  }

  SECTION("framing errors are 400") {
    // Malformed JSON.
    CHECK(StatusOf(RoundTrip(port, PostMcp("{not json", "application/json"))) == 400);
    // Valid JSON that is neither an object nor an array.
    CHECK(StatusOf(RoundTrip(port, PostMcp("123", "application/json"))) == 400);
    // Empty body.
    CHECK(StatusOf(RoundTrip(port, PostMcp("", "application/json"))) == 400);
  }

  SECTION("GET /mcp opens an SSE stream with a keepalive comment") {
    const int fd = ConnectTo(port, 2);  // short recv timeout: the stream stays open
    REQUIRE(fd >= 0);
    REQUIRE(SendAll(fd, "GET /mcp HTTP/1.1\r\nHost: x\r\n\r\n"));
    const std::string resp = ReadAvailable(fd);
    close(fd);
    CHECK(StatusOf(resp) == 200);
    CHECK(HasHeader(resp, "Content-Type: text/event-stream"));
    CHECK(resp.find(": keepalive") != std::string::npos);
  }

  SECTION("client disconnect mid tools/call does not crash the server") {
    for (int i = 0; i < 8; ++i) {
      const int fd = ConnectTo(port, 5);
      REQUIRE(fd >= 0);
      SendAll(
          fd,
          PostMcp(
              R"({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"find_routes","arguments":{"departure":"KJFK","arrival":"KLAX"}}})",
              "application/json"));
      close(fd);  // gone before the offloaded dispatch completes
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const std::string resp = RoundTrip(
        port, PostMcp(R"({"jsonrpc":"2.0","id":2,"method":"tools/list"})", "application/json"));
    CHECK(StatusOf(resp) == 200);
  }
}

#endif  // _WIN32
