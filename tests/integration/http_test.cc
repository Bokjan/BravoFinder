// SPDX-License-Identifier: MIT
// http_test.cc — end-to-end coverage of the bf-http transport: a real
// libuv server on a loopback port, driven over an actual socket, exercising the
// hardening paths llhttp does not handle. This is where the "does the whole
// thing behave" checks live (see docs/http-service): success shapes, the
// 400/404/422
// status mapping, an oversized body (413), chunked rejection, keep-alive with
// several requests on one connection, and a client that disconnects while the
// route computation is still running (the liveness guard must not crash).
//
// The client uses POSIX sockets, so the whole suite is skipped on Windows (the
// socket-free handler coverage in http_handlers_test.cc runs everywhere).

#include <catch2/catch_test_macros.hpp>

#if defined(_WIN32)

TEST_CASE("http server e2e: skipped on Windows", "[integration][http]") {
  SKIP("HTTP end-to-end test uses POSIX sockets; not run on Windows");
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
#include <future>
#include <string>
#include <thread>

#include "conn.h"
#include "core/version.h"
#include "io/cache/bfdb_inventory.h"
#include "rapidjson/document.h"
#include "registry.h"
#include "router.h"
#include "server.h"
#include "test_bfdb.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace {

using bf::test::NavDataDir;

// A bf-http server on its own loop thread, bound to a loopback port. The port is
// handed back through a promise once Listen succeeds. The destructor lets any
// in-flight work and client disconnects drain, then stops the loop and joins.
class ServerHarness {
 public:
  ServerHarness(bf::service::NavDatabaseRegistry& registry, const bf::http_server::Limits& limits)
      : registry_(registry), limits_(limits) {
    std::promise<int> port_promise;
    std::future<int> port_future = port_promise.get_future();
    thread_ = std::thread([this, p = std::move(port_promise)]() mutable { RunLoop(std::move(p)); });
    port_ = port_future.get();
  }

  ~ServerHarness() {
    // Ask the loop thread to stop accepting (close the listener + the async).
    // The loop then drains any in-flight computation and client disconnects on
    // its own and returns from uv_run once no handle or queued work remains, so
    // every Connection frees itself normally — no fixed-delay settle to guess.
    uv_async_send(&stop_);
    thread_.join();
  }

  int port() const { return port_; }

 private:
  void RunLoop(std::promise<int> port_promise) {
    uv_loop_init(&loop_);
    uv_async_init(&loop_, &stop_, [](uv_async_t* a) {
      // Runs on the loop thread: stop accepting, then close the async itself.
      // uv_run(UV_RUN_DEFAULT) returns once the listener, this async, all
      // Connections, and all queued threadpool work have drained.
      auto* server = static_cast<bf::http_server::Server*>(a->data);
      server->Close();
      uv_close(reinterpret_cast<uv_handle_t*>(a), nullptr);
    });
    bf::http::Router router(registry_, &loop_);
    bf::http_server::Server server(&loop_, router, limits_);
    // The async callback needs the server to close its listener. Safe to set
    // before the send: the destructor cannot fire the async until the ctor has
    // returned, which is after port_promise is fulfilled below.
    stop_.data = &server;
    const int r = server.Listen("127.0.0.1", 0);
    port_promise.set_value(r == 0 ? server.BoundPort() : -1);
    uv_run(&loop_, UV_RUN_DEFAULT);
    // Belt and braces: close any handle still open (there should be none after a
    // graceful drain) and flush, then close the loop.
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

int ConnectTo(int port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  timeval tv{15, 0};  // generous: the first request opens the ~1.5s cache
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

// Read one full HTTP response (status line + headers + Content-Length body).
// Returns what was received; empty on immediate failure.
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
      break;  // EOF or timeout
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

int StatusOf(const std::string& response) {
  const std::string prefix = "HTTP/1.1 ";
  if (response.rfind(prefix, 0) != 0) {
    return -1;
  }
  return std::stoi(response.substr(prefix.size(), 3));
}

std::string BodyOf(const std::string& response) {
  const size_t end = response.find("\r\n\r\n");
  if (end == std::string::npos) {
    return "";
  }
  return response.substr(end + 4);
}

// Whether the response header block carries "Connection: <token>".
bool HasConnection(const std::string& response, const std::string& token) {
  const size_t end = response.find("\r\n\r\n");
  const std::string headers = response.substr(0, end == std::string::npos ? response.size() : end);
  return headers.find("Connection: " + token) != std::string::npos;
}

// True if the response header block carries an X-Elapsed-Ms header; if so, its
// numeric value is written to *value.
bool HasElapsedHeader(const std::string& response, unsigned long* value) {
  const size_t end = response.find("\r\n\r\n");
  const std::string headers = response.substr(0, end == std::string::npos ? response.size() : end);
  const std::string key = "X-Elapsed-Ms: ";
  const size_t pos = headers.find(key);
  if (pos == std::string::npos) {
    return false;
  }
  if (value != nullptr) {
    *value = std::stoul(headers.substr(pos + key.size()));
  }
  return true;
}

std::string Get(const std::string& path, bool keep_alive) {
  std::string r = "GET " + path + " HTTP/1.1\r\nHost: x\r\n";
  r += keep_alive ? "Connection: keep-alive\r\n" : "Connection: close\r\n";
  r += "\r\n";
  return r;
}

std::string Post(const std::string& path, const std::string& body, const std::string& extra) {
  std::string r = "POST " + path + " HTTP/1.1\r\nHost: x\r\nConnection: close\r\n";
  r += extra;
  r += "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
  r += body;
  return r;
}

std::string PostKeepAlive(const std::string& path, const std::string& body) {
  std::string r = "POST " + path + " HTTP/1.1\r\nHost: x\r\nConnection: keep-alive\r\n";
  r += "Content-Length: " + std::to_string(body.size()) + "\r\n\r\n";
  r += body;
  return r;
}

// One request over a fresh connection; returns the full response.
std::string RoundTrip(int port, const std::string& request) {
  const int fd = ConnectTo(port);
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

}  // namespace

TEST_CASE("http server end-to-end over a loopback socket", "[integration][http]") {
  signal(SIGPIPE, SIG_IGN);  // writing to a peer-closed socket must not kill us

  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(bf::test::EnsureBfdb());
  if (!inventory || inventory.value().empty()) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::service::NavDatabaseRegistry registry(std::move(inventory.value()));

  bf::http_server::Limits limits;
  limits.max_body_bytes = 1024;  // small, so a modest body triggers 413
  ServerHarness harness(registry, limits);
  REQUIRE(harness.port() > 0);
  const int port = harness.port();

  SECTION("probes and cycle list") {
    const std::string health = RoundTrip(port, Get("/healthz", false));
    CHECK(StatusOf(health) == 200);
    CHECK(BodyOf(health) == R"({"status":"ok"})");

    const std::string ready = RoundTrip(port, Get("/readyz", false));
    CHECK(StatusOf(ready) == 200);
    CHECK(BodyOf(ready) == R"({"status":"ready"})");

    const std::string cycles = RoundTrip(port, Get("/v1/cycles", false));
    CHECK(StatusOf(cycles) == 200);
    CHECK(BodyOf(cycles).find("\"cycles\"") != std::string::npos);

    const std::string version = RoundTrip(port, Get("/v1/version", false));
    CHECK(StatusOf(version) == 200);
    rapidjson::Document ver_doc;
    ver_doc.Parse(BodyOf(version).c_str());
    REQUIRE_FALSE(ver_doc.HasParseError());
    REQUIRE(ver_doc.IsObject());
    REQUIRE(ver_doc.HasMember("version"));
    REQUIRE(ver_doc["version"].IsString());
    CHECK(std::string(ver_doc["version"].GetString()) == bf::kBravoFinderVersion);
  }

  SECTION("find_routes success shape") {
    const std::string resp =
        RoundTrip(port, Post("/v1/routes", R"({"departure":"KJFK","arrival":"KLAX","k":1})", ""));
    REQUIRE(StatusOf(resp) == 200);
    rapidjson::Document doc;
    doc.Parse(BodyOf(resp).c_str());
    REQUIRE_FALSE(doc.HasParseError());
    REQUIRE(doc.IsArray());
    REQUIRE(doc.Size() >= 1);
    const rapidjson::Value& route = doc[0];
    CHECK(route.HasMember("points"));
    CHECK(route.HasMember("legs"));
    CHECK(route["points"].IsArray());
    // N points, N-1 legs.
    CHECK(route["points"].Size() == route["legs"].Size() + 1);
  }

  SECTION("status mapping: 400 / 422 / 404") {
    CHECK(StatusOf(RoundTrip(port, Post("/v1/routes", R"({"departure":"KJFK"})", ""))) == 400);
    CHECK(StatusOf(RoundTrip(
              port, Post("/v1/routes", R"({"departure":"ZZ_NOPE_ZZ","arrival":"KLAX"})", ""))) ==
          422);
    CHECK(StatusOf(RoundTrip(port, Get("/v1/nope", false))) == 404);
  }

  SECTION("X-Elapsed-Ms header on success, absent on error") {
    // A successful route reports its compute cost as an X-Elapsed-Ms header,
    // with a sane value (well under a minute).
    const std::string ok =
        RoundTrip(port, Post("/v1/routes", R"({"departure":"KJFK","arrival":"KLAX","k":1})", ""));
    REQUIRE(StatusOf(ok) == 200);
    unsigned long ms = 0;
    REQUIRE(HasElapsedHeader(ok, &ms));
    CHECK(ms <= 60000);
    // A 400 (missing arrival) carries no timing header.
    const std::string bad = RoundTrip(port, Post("/v1/routes", R"({"departure":"KJFK"})", ""));
    REQUIRE(StatusOf(bad) == 400);
    CHECK_FALSE(HasElapsedHeader(bad, nullptr));
  }

  SECTION("oversized body is rejected with 413") {
    const std::string big(4096, 'a');  // exceeds the 1024-byte limit
    const std::string resp = RoundTrip(port, Post("/v1/routes", big, ""));
    CHECK(StatusOf(resp) == 413);
  }

  SECTION("chunked transfer-encoding is rejected with 400") {
    // Transfer-Encoding present => refuse and close (no chunked parsing).
    std::string req = "POST /v1/routes HTTP/1.1\r\nHost: x\r\nTransfer-Encoding: chunked\r\n\r\n";
    req += "5\r\nhello\r\n0\r\n\r\n";
    const std::string resp = RoundTrip(port, req);
    CHECK(StatusOf(resp) == 400);
  }

  SECTION("keep-alive serves several requests on one connection") {
    const int fd = ConnectTo(port);
    REQUIRE(fd >= 0);
    for (int i = 0; i < 3; ++i) {
      REQUIRE(SendAll(fd, Get("/healthz", true)));
      const std::string resp = ReadResponse(fd);
      CHECK(StatusOf(resp) == 200);
    }
    close(fd);
  }

  SECTION("client disconnect mid-computation does not crash the server") {
    // Fire several route requests and close each socket immediately, before the
    // worker finishes. The liveness guard must drop each response cleanly.
    for (int i = 0; i < 8; ++i) {
      const int fd = ConnectTo(port);
      REQUIRE(fd >= 0);
      SendAll(fd, Post("/v1/routes", R"({"departure":"KJFK","arrival":"KLAX","k":10})", ""));
      close(fd);  // gone before the ~10-30 ms computation completes
    }
    // The server is still healthy afterwards.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const std::string health = RoundTrip(port, Get("/healthz", false));
    CHECK(StatusOf(health) == 200);
  }

  SECTION("bytes while awaiting a response force Connection: close") {
    // Pipeline across reads: start a keep-alive route (offloaded), then send a
    // second request before the response is written. The second request's bytes
    // are dropped and the first response must close the connection so keep-alive
    // cannot desync onto the discarded request.
    const int fd = ConnectTo(port);
    REQUIRE(fd >= 0);
    REQUIRE(SendAll(
        fd, PostKeepAlive("/v1/routes", R"({"departure":"KJFK","arrival":"KLAX","k":10})")));
    REQUIRE(SendAll(fd, Get("/healthz", true)));
    const std::string resp = ReadResponse(fd);
    REQUIRE(StatusOf(resp) == 200);
    CHECK(HasConnection(resp, "close"));
    close(fd);
  }
}

// Hardening paths that reject or close a connection before any request reaches
// routing, so they need no navigation data (an empty registry suffices) and run
// even where navdata/ is absent. Covers the fixed header caps (431) and the
// idle/slowloris timeout. The 431 sections also drive the reject_status_ write
// path in Connection::OnRead (a parser callback sets the status and returns -1,
// then OnRead emits the error response) -- otherwise reached only indirectly.
TEST_CASE("http hardening: header limits and idle timeout", "[integration][http]") {
  signal(SIGPIPE, SIG_IGN);

  // Empty registry: none of these requests reach routing, so no data is needed.
  bf::service::NavDatabaseRegistry registry(bf::BfdbInventory{});

  bf::http_server::Limits limits;
  limits.io_timeout_ms = 300;  // short, so the idle-timeout section stays fast
  ServerHarness harness(registry, limits);
  REQUIRE(harness.port() > 0);
  const int port = harness.port();

  SECTION("too many headers are rejected with 431") {
    std::string req = "GET /healthz HTTP/1.1\r\nHost: x\r\n";
    for (int i = 0; i < 200; ++i) {  // exceeds kMaxHeaderCount (100)
      req += "X-Pad-" + std::to_string(i) + ": v\r\n";
    }
    req += "\r\n";
    CHECK(StatusOf(RoundTrip(port, req)) == 431);
  }

  SECTION("an oversized header block is rejected with 431") {
    std::string req = "GET /healthz HTTP/1.1\r\nHost: x\r\n";
    req += "X-Big: " + std::string(64 * 1024, 'a') + "\r\n";  // exceeds kMaxHeaderBytes (32 KiB)
    req += "\r\n";
    CHECK(StatusOf(RoundTrip(port, req)) == 431);
  }

  SECTION("an oversized request-line URL is rejected with 414") {
    // A multi-megabyte request line must trip the URL cap incrementally, not grow
    // url_ unbounded until the (never-arriving) message completes.
    std::string req = "GET /" + std::string(16 * 1024, 'A') + " HTTP/1.1\r\nHost: x\r\n\r\n";
    CHECK(StatusOf(RoundTrip(port, req)) == 414);
  }

  SECTION("trailing bytes after a complete request force a close (no pipelining)") {
    // A complete keep-alive request with extra bytes appended (a would-be second,
    // pipelined request). We do not pipeline: the first request is answered but
    // the connection is closed so the un-processed bytes are not silently lost.
    std::string req = "GET /healthz HTTP/1.1\r\nHost: x\r\nConnection: keep-alive\r\n\r\n";
    req += "GET /healthz HTTP/1.1\r\nHost: x\r\n\r\n";  // trailing pipelined request
    const std::string resp = RoundTrip(port, req);
    CHECK(StatusOf(resp) == 200);
    CHECK(HasConnection(resp, "close"));  // forced close despite the keep-alive ask
  }

  SECTION("an idle connection is closed after the io timeout") {
    const int fd = ConnectTo(port);
    REQUIRE(fd >= 0);
    // A partial request with no terminating CRLFCRLF, then stall. The idle timer
    // must fire; per RFC 9110 §15.5.7 the server answers 408 and then closes, so
    // the client learns why the connection dropped rather than just seeing EOF.
    REQUIRE(SendAll(fd, "GET /healthz HTTP/1.1\r\nHost: x\r\n"));
    const std::string resp = ReadResponse(fd);
    CHECK(StatusOf(resp) == 408);
    CHECK(HasConnection(resp, "close"));
    close(fd);
  }

  SECTION("Content-Length over the body cap is rejected with 413 before the body") {
    // Declare a body larger than max_body_bytes (default 1 MiB here) without
    // sending it: headers-complete must 413 immediately.
    std::string req = "POST /healthz HTTP/1.1\r\nHost: x\r\nContent-Length: 2000000\r\n\r\n";
    CHECK(StatusOf(RoundTrip(port, req)) == 413);
  }

  SECTION("Expect: 100-continue is rejected with 417") {
    std::string req =
        "POST /healthz HTTP/1.1\r\nHost: x\r\nContent-Length: 0\r\nExpect: 100-continue\r\n\r\n";
    CHECK(StatusOf(RoundTrip(port, req)) == 417);
  }

  SECTION("request deadline fires even when idle timer is kept reset") {
    // io_timeout is long enough that per-chunk idle resets would never fire;
    // request_timeout is short so a drip across message-begin still 408s.
    bf::service::NavDatabaseRegistry registry2(bf::BfdbInventory{});
    bf::http_server::Limits tight;
    tight.io_timeout_ms = 2000;
    tight.request_timeout_ms = 400;
    ServerHarness h2(registry2, tight);
    REQUIRE(h2.port() > 0);
    const int fd = ConnectTo(h2.port());
    REQUIRE(fd >= 0);
    REQUIRE(SendAll(fd, "POST /healthz HTTP/1.1\r\n"));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    REQUIRE(SendAll(fd, "Host: x\r\n"));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    REQUIRE(SendAll(fd, "Content-Length: 0\r\n\r\n"));
    const std::string resp = ReadResponse(fd);
    CHECK(StatusOf(resp) == 408);
    CHECK(HasConnection(resp, "close"));
    close(fd);
  }

  SECTION("max_connections sheds additional accepts") {
    bf::service::NavDatabaseRegistry registry2(bf::BfdbInventory{});
    bf::http_server::Limits one;
    one.max_connections = 1;
    one.io_timeout_ms = 5000;
    ServerHarness h2(registry2, one);
    REQUIRE(h2.port() > 0);
    const int held = ConnectTo(h2.port());
    REQUIRE(held >= 0);
    REQUIRE(SendAll(held, Get("/healthz", true)));
    REQUIRE(StatusOf(ReadResponse(held)) == 200);
    // First connection is still live (keep-alive). A second accept is closed
    // without serving a response.
    const int extra = ConnectTo(h2.port());
    REQUIRE(extra >= 0);
    REQUIRE(SendAll(extra, Get("/healthz", false)));
    const std::string resp = ReadResponse(extra);
    CHECK(resp.empty());
    close(extra);
    close(held);
  }
}

#endif  // _WIN32
