// SPDX-License-Identifier: MIT
// router.cc — endpoint table, ?cycle= parsing, and status selection for bf-http.

#include "router.h"

#include <cassert>
#include <charconv>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "api_keys.h"
#include "conn.h"  // http_server::Connection
#include "core/version.h"
#include "io/cache/bfdb_inventory.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "render.h"  // bf::service::JsonError
#include "work.h"    // http_server::QueueWork, WorkResult

namespace bf::http {

namespace {

// Maximum in-flight offloaded work items before new requests are shed with 503.
// libuv's default threadpool is 4 threads, so this allows a deep-but-bounded
// queue; each item pins a Connection (with its 64 KiB read buffer) + args
// Document, so the cap also bounds worst-case memory under a request flood.
constexpr int kMaxInflightWork = 256;

// The HTTP path each shared handler is exposed under (see docs/http-service).
// Keyed by the stable bf::service handler name (api_keys.h).
struct Route {
  std::string_view name;
  const char* path;
};
const Route kRoutes[] = {
    {bf::service::kHandlerFindRoutes, "/v1/routes"},
    {bf::service::kHandlerParseRoute, "/v1/parse-route"},
    {bf::service::kHandlerLookupWaypoints, "/v1/waypoints"},
    {bf::service::kHandlerLookupAirports, "/v1/airports"},
    {bf::service::kHandlerLookupProcedures, "/v1/procedures"},
    {bf::service::kHandlerLookupProcedureLegs, "/v1/procedure-legs"},
    {bf::service::kHandlerLookupAirways, "/v1/airways"},
    {bf::service::kHandlerLookupNavaidDetail, "/v1/navaid-detail"},
    {bf::service::kHandlerLookupHolds, "/v1/holds"},
};

// Parse an optional ?cycle=NNNN out of the raw query string. Returns false if a
// cycle parameter is present but not a valid non-negative integer; true (with
// *out possibly still nullopt) when absent or well-formed.
bool ParseCycle(const std::string& query, std::optional<uint32_t>* out) {
  *out = std::nullopt;
  size_t i = 0;
  while (i < query.size()) {
    const size_t amp = query.find('&', i);
    const std::string_view token(query.data() + i,
                                 (amp == std::string::npos ? query.size() : amp) - i);
    const size_t eq = token.find('=');
    if (eq != std::string_view::npos && token.substr(0, eq) == "cycle") {
      const std::string_view value = token.substr(eq + 1);
      uint32_t parsed = 0;
      const char* begin = value.data();
      const char* end = value.data() + value.size();
      const auto [ptr, ec] = std::from_chars(begin, end, parsed);
      if (ec != std::errc() || ptr != end || value.empty()) {
        return false;
      }
      *out = parsed;
      return true;
    }
    if (amp == std::string::npos) {
      break;
    }
    i = amp + 1;
  }
  return true;
}

// Build a small {"status":"<value>"} JSON object with a Writer rather than a
// hand-rolled string literal (project rule: no hand-rolled JSON). Used by the
// liveness/readiness endpoints, whose bodies are otherwise trivial constants.
std::string StatusJson(const char* value) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("status");
  writer.String(value);
  writer.EndObject();
  return buffer.GetString();
}

// Program version for consumers (display / compatibility checks). Loop-thread
// only — no database touch. Machine-readable semver only; the multi-line LGPL
// banner stays on --version (see bf::service::VersionBanner).
std::string VersionJson() {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("version");
  writer.String(bf::kBravoFinderVersion);
  writer.EndObject();
  return buffer.GetString();
}

// Readiness handler: reached only when registry.Get(latest) succeeded, so the
// server can open and serve the newest cycle. A resolve failure is turned into
// 503 by the cycle_error_status below, never reaching this handler.
bf::service::HandlerResult ReadyHandler(const rapidjson::Value& /*args*/,
                                        const bf::NavDatabase& /*db*/) {
  return {.body = StatusJson("ready"), .status = http_server::kStatusOk};
}

// Build the offloaded-work closure: resolve the database for `cycle` then run
// `handler(args)`, projecting the result into a transport-neutral WorkResult. A
// resolve failure becomes `cycle_error_status`. The closure owns `args` via a
// shared_ptr so it stays copyable (std::function requires a copyable target),
// and captures `registry` by reference (it outlives the Router and all work).
std::function<http_server::WorkResult()> MakeWork(bf::service::NavDatabaseRegistry& registry,
                                                  bf::service::QueryHandler handler,
                                                  std::shared_ptr<rapidjson::Document> args,
                                                  std::optional<uint32_t> cycle,
                                                  int cycle_error_status) {
  return [&registry, handler = std::move(handler), args = std::move(args), cycle,
          cycle_error_status]() -> http_server::WorkResult {
    bf::Result<const bf::NavDatabase*> db = registry.Get(cycle);
    if (!db) {
      http_server::WorkResult err;
      err.status = cycle_error_status;
      err.body = bf::service::JsonError(db.error().message);
      return err;
    }
    bf::service::HandlerResult r = handler(*args, *db.value());
    http_server::WorkResult out;
    out.status = r.status;
    out.body = std::move(r.body);
    out.elapsed_ms = r.elapsed_ms;
    return out;
  };
}

}  // namespace

Router::Router(bf::service::NavDatabaseRegistry& registry, uv_loop_t* loop)
    : registry_(registry), loop_(loop) {
  // Index the shared handlers by name, then bind each to its HTTP path.
  std::unordered_map<std::string, bf::service::QueryHandler> by_name;
  for (bf::service::NamedHandler& nh : bf::service::MakeHandlers()) {
    by_name.emplace(std::move(nh.name), std::move(nh.handler));
  }
  for (const Route& r : kRoutes) {
    auto it = by_name.find(std::string(r.name));
    // Every path we expose must map to a handler registered by
    // bf::service::MakeHandlers(); a name drift there would otherwise silently
    // drop the endpoint. Catch it in debug builds.
    assert(it != by_name.end() && "HTTP route name has no matching bf::service handler");
    if (it != by_name.end()) {
      routes_.emplace(r.path, std::move(it->second));
    }
  }
}

void Router::Handle(std::shared_ptr<http_server::Connection> conn,
                    const http_server::HttpRequest& req) {
  // Liveness: the process is up, so always 200 -- no database touched.
  if (req.method == "GET" && req.path == "/healthz") {
    conn->WriteResponse(http_server::kStatusOk, StatusJson("ok"), req.keep_alive);
    return;
  }
  // Program version for consumers — loop-thread, no database.
  if (req.method == "GET" && req.path == "/v1/version") {
    conn->WriteResponse(http_server::kStatusOk, VersionJson(), req.keep_alive);
    return;
  }
  // The cycle list is an in-memory read of the (immutable) inventory.
  if (req.method == "GET" && req.path == "/v1/cycles") {
    conn->WriteResponse(http_server::kStatusOk, SerializeCycles(), req.keep_alive);
    return;
  }
  // Readiness: opening the latest cycle may do disk I/O, so offload it; 503 if
  // it cannot be resolved.
  if (req.method == "GET" && req.path == "/readyz") {
    if (inflight_.load(std::memory_order_relaxed) >= kMaxInflightWork) {
      conn->WriteResponse(http_server::kStatusServiceUnavailable,
                          http_server::JsonError("server busy"), req.keep_alive);
      return;
    }
    auto empty = std::make_shared<rapidjson::Document>();
    empty->SetObject();
    http_server::QueueWork(std::move(conn), loop_,
                           MakeWork(registry_, ReadyHandler, std::move(empty), std::nullopt,
                                    http_server::kStatusServiceUnavailable),
                           req.keep_alive, inflight_);
    return;
  }

  // Query endpoints: POST + JSON body. An unmatched (method, path) is 404.
  auto it = routes_.find(req.path);
  if (req.method != "POST" || it == routes_.end()) {
    conn->WriteResponse(http_server::kStatusNotFound, http_server::JsonError("not found"),
                        req.keep_alive);
    return;
  }
  // Shed load before doing any per-request work if the offload queue is full.
  if (inflight_.load(std::memory_order_relaxed) >= kMaxInflightWork) {
    conn->WriteResponse(http_server::kStatusServiceUnavailable,
                        http_server::JsonError("server busy"), req.keep_alive);
    return;
  }
  // ?cycle=NNNN selects the database; malformed cycle is a 400 before any work.
  std::optional<uint32_t> cycle;
  if (!ParseCycle(req.query, &cycle)) {
    conn->WriteResponse(http_server::kStatusBadRequest,
                        http_server::JsonError("cycle must be a non-negative integer"),
                        req.keep_alive);
    return;
  }
  // The body is the handler's arguments object. Empty body => empty object;
  // malformed JSON (or a non-object) is a 400 before any work. Parse iteratively
  // (kParseIterativeFlag): the body is untrusted and up to 1 MiB, and rapidjson's
  // default recursive-descent parser would blow the C++ stack on a deeply nested
  // "[[[[..." payload, crashing the whole process. Parse over (data, size) rather
  // than a C string so an embedded NUL cannot truncate the body.
  auto args = std::make_shared<rapidjson::Document>();
  if (req.body.empty()) {
    args->SetObject();
  } else {
    args->Parse<rapidjson::kParseIterativeFlag>(req.body.data(), req.body.size());
    if (args->HasParseError() || !args->IsObject()) {
      conn->WriteResponse(http_server::kStatusBadRequest,
                          http_server::JsonError("request body must be a JSON object"),
                          req.keep_alive);
      return;
    }
  }
  // Offload: an unknown/unserved cycle is the client's error (400).
  http_server::QueueWork(
      std::move(conn), loop_,
      MakeWork(registry_, it->second, std::move(args), cycle, http_server::kStatusBadRequest),
      req.keep_alive, inflight_);
}

std::string Router::SerializeCycles() const {
  const BfdbInventory& inv = registry_.inventory();
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("cycles");
  writer.StartArray();
  // Entries are sorted ascending; present newest first for readability.
  const auto& entries = inv.entries();
  for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
    writer.StartObject();
    writer.Key("cycle");
    writer.Uint(it->cycle);
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();
  return buffer.GetString();
}

}  // namespace bf::http
