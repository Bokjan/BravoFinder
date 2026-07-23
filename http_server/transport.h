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

namespace bf::http_server {

class Connection;

// A list of extra response headers (name, value), beyond the framing headers
// (Content-Type / Content-Length / Connection / Date) the core always writes.
using Headers = std::vector<std::pair<std::string, std::string>>;

// Transport limits/timeouts. Body size and the idle timeout are CLI-tunable;
// the header caps are fixed hardening constants (see conn.cc).
struct Limits {
  size_t max_body_bytes = 1u << 20;  // 1 MiB request body cap (--max-body)
  uint64_t io_timeout_ms = 30'000;   // header/body read + idle keep-alive (--io-timeout)
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

  // The first value of a request header by (case-insensitive) name, or "" if
  // absent. Linear scan: the header set is tiny and bounded by kMaxHeaderCount.
  std::string Header(const std::string& lower_name) const;
};

// The result of an offloaded unit of work, produced on a worker thread and
// written back on the loop thread. status is an HTTP status code; body is the
// (already-serialized) response body; elapsed_ms is the compute cost surfaced as
// an X-Elapsed-Ms header (0 => header omitted). content_type overrides the
// response Content-Type (empty => the default "application/json", used by REST);
// extra_headers are appended verbatim (e.g. an MCP Mcp-Session-Id). Transport-
// local by design so the core does not depend on bf_service_lib.
struct WorkResult {
  int status = 200;
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
// the message is auto-escaped. Used for the transport's own rejects (413/431/
// 414/400/503); query-layer errors use bf::service::JsonError.
std::string JsonError(const std::string& message);

}  // namespace bf::http_server
