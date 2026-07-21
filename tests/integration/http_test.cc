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
  ServerHarness(bf::service::NavDatabaseRegistry& registry, const bf::http::Limits& limits)
      : registry_(registry), limits_(limits) {
    std::promise<int> port_promise;
    std::future<int> port_future = port_promise.get_future();
    thread_ = std::thread([this, p = std::move(port_promise)]() mutable { RunLoop(std::move(p)); });
    port_ = port_future.get();
  }

  ~ServerHarness() {
    // Give worker threads time to finish in-flight computations and the loop to
    // process client disconnects, so every Connection frees itself normally
    // before we force the loop down (avoids leaking a half-closed handle).
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    uv_async_send(&stop_);
    thread_.join();
  }

  int port() const { return port_; }

 private:
  void RunLoop(std::promise<int> port_promise) {
    uv_loop_init(&loop_);
    uv_async_init(&loop_, &stop_, [](uv_async_t* a) { uv_stop(a->loop); });
    bf::http::Router router(registry_, &loop_);
    bf::http::Server server(&loop_, router, limits_);
    const int r = server.Listen("127.0.0.1", 0);
    port_promise.set_value(r == 0 ? server.BoundPort() : -1);
    uv_run(&loop_, UV_RUN_DEFAULT);
    // Close whatever handles remain (listener + async) and flush, then close the
    // loop. Client connections have already drained during the destructor's
    // settle, so nothing shared_ptr-owned is force-closed here.
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
  bf::http::Limits limits_;
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

  bf::http::Limits limits;
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
}

#endif  // _WIN32
