// SPDX-License-Identifier: MIT
// transport.h — the transport-neutral surface of the shared HTTP core.
//
// http_server/ is the hand-rolled HTTP/1.1 transport (libuv event loop + llhttp
// parser + the safety hardening llhttp does not do), factored out of apps/http
// so both the REST server (apps/http) and the MCP-over-HTTP transport (apps/mcp)
// can reuse it. It is deliberately free of any query/JSON-schema knowledge: it
// knows sockets, requests, responses, and how to offload work -- nothing about
// what a request means. A consumer implements RequestHandler to give a request
// meaning, and returns a WorkResult from offloaded work.
//
// This header holds the small value types shared across the core and its
// consumers: the parsed request, the transport limits, the offloaded-work
// result, the handler interface, and a JSON error helper for transport-level
// rejects. The connection state machine is in conn.{h,cc}, the listener in
// server.{h,cc}, and the threadpool offload in work.{h,cc}.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "http_status.h"  // kStatusOk

namespace bf::http_server {

class Connection;

// Default request read + idle keep-alive timeout, in seconds (overridable via
// --io-timeout). The Limits struct stores it as milliseconds.
inline constexpr int kDefaultIoTimeoutSec = 30;

// Default bind port for bf-http (--port).
inline constexpr int kDefaultPort = 8080;

// Default bind port for bf-mcp --transport http (--port). Distinct from
// kDefaultPort so bf-http and bf-mcp can run side-by-side on one host.
inline constexpr int kDefaultMcpPort = 8081;

// Default request body cap, in bytes (--max-body). 1 MiB.
inline constexpr size_t kDefaultMaxBodyBytes = 1u << 20;

// Default cap on simultaneously open connections (accept path). Beyond this the
// server accepts then immediately closes so the backlog does not stall forever.
inline constexpr size_t kDefaultMaxConnections = 1024;

// Default hard deadline for assembling one request (from message-begin), in
// seconds. Distinct from the idle timer: a client that dribbles bytes just under
// io_timeout_ms can hold a connection for ~body_cap * idle without this.
inline constexpr int kDefaultRequestTimeoutSec = 120;

// Seconds-to-milliseconds scale for Limits::io_timeout_ms.
inline constexpr uint64_t kMsPerSec = 1000;

// A list of extra response headers (name, value), beyond the framing headers
// (Content-Type / Content-Length / Connection / Date) the core always writes.
using Headers = std::vector<std::pair<std::string, std::string>>;

// Transport limits/timeouts. Body size and the idle timeout are CLI-tunable;
// the header caps are fixed hardening constants (see conn.cc).
struct Limits {
  size_t max_body_bytes = kDefaultMaxBodyBytes;  // 1 MiB request body cap (--max-body)
  uint64_t io_timeout_ms = static_cast<uint64_t>(kDefaultIoTimeoutSec) *
                           kMsPerSec;  // header/body read + idle keep-alive (--io-timeout)
  // Hard wall-clock budget to finish reading one request after its first byte
  // of the message (OnMessageBegin). 0 disables the deadline (idle timeout only).
  uint64_t request_timeout_ms = static_cast<uint64_t>(kDefaultRequestTimeoutSec) * kMsPerSec;
  // Simultaneous live connections. 0 = unlimited (tests / specialised embeds).
  size_t max_connections = kDefaultMaxConnections;
};

// A fully-parsed HTTP request handed to the RequestHandler. method/path are what
// routing keys off; query carries the raw string after '?'; headers holds the
// request headers (lower-cased names) a consumer may need (e.g. Accept for SSE
// content negotiation); body is the raw request body (may be empty).
struct HttpRequest {
  std::string method;
  std::string path;
  std::string query;
  std::string body;
  Headers headers;
  bool keep_alive = false;

  // The first value of a request header by name, or "" if absent. Comparison is
  // ASCII case-insensitive (request header names are stored lower-cased by the
  // parser, but callers may pass mixed case). Linear scan: the header set is
  // tiny and bounded by kMaxHeaderCount.
  std::string Header(const std::string& name) const;
};

// The result of an offloaded unit of work, produced on a worker thread and
// written back on the loop thread. status is an HTTP status code; body is the
// (already-serialized) response body; elapsed_ms is the compute cost surfaced as
// an X-Elapsed-Ms header (0 => header omitted). content_type overrides the
// response Content-Type (empty => the default "application/json", used by REST);
// extra_headers are appended after framing (e.g. an MCP Mcp-Session-Id). Framing
// names (Content-Length / Content-Type / Connection / Date / Transfer-Encoding)
// in extra_headers are dropped by the transport so handlers cannot override them.
// WorkResult is transport-local by design so the core does not depend on
// bf_service_lib.
struct WorkResult {
  int status = kStatusOk;
  std::string body;
  uint32_t elapsed_ms = 0;
  std::string content_type;  // empty => "application/json"
  Headers extra_headers;
};

// The single point where the transport hands a parsed request to its consumer.
// Called on the libuv loop thread. The handler answers inline (via
// conn->WriteResponse / conn->BeginStream) or offloads to the threadpool (via
// QueueWork) and writes back from the completion callback. The handler must
// outlive every connection routed to it.
class RequestHandler {
 public:
  virtual ~RequestHandler() = default;
  virtual void Handle(std::shared_ptr<Connection> conn, const HttpRequest& req) = 0;
};

// Build an error JSON payload {"error":"<message>"} with RapidJSON's Writer so
// the message is auto-escaped. Used for the transport's own rejects (413/417/
// 431/414/400/503); query-layer errors use bf::service::JsonError.
std::string JsonError(const std::string& message);

}  // namespace bf::http_server
