// SPDX-License-Identifier: MIT
// mcp_http.cc — the MCP-over-HTTP (Streamable HTTP, 2025-03-26) transport.
//
// Serves the single /mcp endpoint over the shared HTTP core. POST carries a
// JSON-RPC request (single or batch), which -- after framing validation on the
// loop thread -- is offloaded to the threadpool and dispatched by the shared
// Dispatcher, then written back on the loop thread as either an application/json
// body or a single SSE event, per the client's Accept header. GET opens an SSE
// stream (a placeholder; no server-push today). DELETE acknowledges a session
// teardown. The transport is stateless-with-id: initialize returns a random
// Mcp-Session-Id that the server does not track.

#include "mcp_http.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <utility>

#include "conn.h"  // http_server::Connection
#include "jsonrpc.h"
#include "rapidjson/document.h"
#include "work.h"  // http_server::QueueWork, WorkResult

namespace bf::mcp {

namespace {

// Maximum in-flight offloaded dispatches before new requests are shed with 503.
// Mirrors the REST router's cap; each item pins a Connection + parsed request.
constexpr int kMaxInflightWork = 256;

// A JSON-RPC error envelope with a null id, for a framing failure where no
// request id can be extracted (a parse error, or a body that is neither a JSON
// object nor an array). Built with the Writer so the message is auto-escaped.
std::string JsonRpcFramingError(int code, const std::string& message) {
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> w(buf);
  w.StartObject();
  w.Key("jsonrpc");
  w.String(jsonrpc::kVersion);
  w.Key("id");
  w.Null();
  w.Key("error");
  w.StartObject();
  w.Key("code");
  w.Int(code);
  w.Key("message");
  w.String(message.c_str(), static_cast<unsigned>(message.size()));
  w.EndObject();
  w.EndObject();
  return buf.GetString();
}

// A random 128-bit session id as 32 lowercase hex chars. Stateless: the server
// returns it on initialize but does not track it (each request is independent,
// since the Dispatcher holds no per-session state).
std::string GenerateSessionId() {
  // A thread_local engine seeded once from random_device, rather than
  // constructing a fresh std::random_device per call: that construction can be
  // slow or even blocking on some libstdc++ configurations. The id is stateless
  // (never tracked server-side) and not a security token, so a per-thread PRNG
  // is ample.
  thread_local std::mt19937_64 engine(std::random_device{}());
  const uint64_t hi = engine();
  const uint64_t lo = engine();
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx", static_cast<unsigned long long>(hi),
                static_cast<unsigned long long>(lo));
  return std::string(buf, 32);
}

// Encode `json` as the data of a single SSE event. Each line gets its own
// "data:" field (SSE folds them back with '\n'), so the frame stays well-formed
// even if the body ever spans multiple lines -- today rapidjson's Writer emits
// compact single-line JSON, but this does not silently break if that changes.
std::string SseData(const std::string& json) {
  std::string out;
  out.reserve(json.size() + 16);
  size_t start = 0;
  while (start <= json.size()) {
    const size_t nl = json.find('\n', start);
    const size_t end = nl == std::string::npos ? json.size() : nl;
    out += "data: ";
    out.append(json, start, end - start);
    out += '\n';
    if (nl == std::string::npos) {
      break;
    }
    start = nl + 1;
  }
  out += '\n';  // blank line terminates the event
  return out;
}

// Whether the request's Accept header opts into an SSE (text/event-stream)
// response. When present, the server replies with a single buffered SSE event;
// otherwise it replies with an application/json body. A plain substring match
// (no q-value / "*/*" handling) is deliberate: MCP clients send an explicit
// "Accept: application/json, text/event-stream", so full content negotiation
// would be dead complexity here.
bool WantsSse(const http_server::HttpRequest& req) {
  return req.Header("accept").find("text/event-stream") != std::string::npos;
}

// Whether a parsed request (single object or batch) is or contains an initialize
// call, so the HTTP transport can attach a fresh Mcp-Session-Id. Session tracking
// is an HTTP concern (the id rides in a header), so this stays in the transport
// rather than the transport-neutral Dispatcher.
bool ContainsInitialize(const rapidjson::Document& req) {
  const auto is_init = [](const rapidjson::Value& v) {
    return v.IsObject() && v.HasMember("method") && v["method"].IsString() &&
           std::strcmp(v["method"].GetString(), "initialize") == 0;
  };
  if (req.IsArray()) {
    for (const rapidjson::Value& item : req.GetArray()) {
      if (is_init(item)) {
        return true;
      }
    }
    return false;
  }
  return is_init(req);
}

// Run the dispatch for one POST body and shape the result into a WorkResult.
// Runs on a worker thread. The Dispatcher handles both a single object and a
// batch (array) -- including the -32600 entry for a non-object batch element --
// so this function is only the HTTP framing: a no-response dispatch (pure
// notification(s)) becomes 202 with an empty body, per Streamable HTTP;
// otherwise the response is framed as JSON or a single SSE event, with a fresh
// Mcp-Session-Id header when the request is or contains an initialize.
http_server::WorkResult BuildMcpWorkResult(const Dispatcher& dispatcher,
                                           const rapidjson::Document& req, bool wants_sse) {
  const Dispatcher::Response r = dispatcher.Dispatch(req);
  if (!r.has_response) {
    // Pure notification(s): acknowledge with 202 and no body.
    http_server::WorkResult out;
    out.status = http_server::kStatusAccepted;
    return out;
  }

  http_server::WorkResult out;
  out.status = http_server::kStatusOk;
  if (wants_sse) {
    out.content_type = "text/event-stream";
    out.body = SseData(r.body);
  } else {
    out.content_type = "application/json";
    out.body = r.body;
  }
  if (ContainsInitialize(req)) {
    out.extra_headers.emplace_back("Mcp-Session-Id", GenerateSessionId());
  }
  return out;
}

}  // namespace

void McpHttpHandler::Handle(std::shared_ptr<http_server::Connection> conn,
                            const http_server::HttpRequest& req) {
  // Strip a single trailing '/' so /mcp/ matches /mcp. Leave "/" alone and do
  // not percent-decode — path match stays literal otherwise.
  std::string path = req.path;
  if (path.size() > 1 && path.back() == '/') {
    path.pop_back();
  }
  if (path != "/mcp") {
    conn->WriteResponse(http_server::kStatusNotFound, http_server::JsonError("not found"),
                        req.keep_alive);
    return;
  }
  if (req.method == "POST") {
    HandlePost(std::move(conn), req);
    return;
  }
  if (req.method == "GET") {
    // Open an SSE stream. There is no server-push today (no progress events, no
    // notifications), so this is a placeholder: send one keepalive comment and
    // let the idle timer or a client disconnect close the stream.
    conn->BeginStream(http_server::kStatusOk, "text/event-stream", {});
    conn->WriteEvent(": keepalive\n\n");
    return;
  }
  if (req.method == "DELETE") {
    // Stateless: acknowledge the session teardown without tracking anything.
    conn->WriteResponse(http_server::kStatusOk, "", req.keep_alive);
    return;
  }
  conn->WriteResponse(http_server::kStatusMethodNotAllowed,
                      http_server::JsonError("method not allowed"), req.keep_alive);
}

void McpHttpHandler::HandlePost(std::shared_ptr<http_server::Connection> conn,
                                const http_server::HttpRequest& req) {
  // Shed load before parsing if the offload queue is full.
  if (inflight_.load(std::memory_order_relaxed) >= kMaxInflightWork) {
    conn->WriteResponse(http_server::kStatusServiceUnavailable,
                        http_server::JsonError("server busy"), req.keep_alive);
    return;
  }
  if (req.body.empty()) {
    conn->WriteResponse(http_server::kStatusBadRequest,
                        JsonRpcFramingError(jsonrpc::kParseError, "empty request body"),
                        req.keep_alive);
    return;
  }
  // Parse the body iteratively (kParseIterativeFlag): it is untrusted and up to
  // the body cap, and rapidjson's recursive parser would blow the C++ stack on a
  // deeply nested payload. Own it via a shared_ptr so the offload closure stays
  // copyable (std::function requires a copyable target).
  auto doc = std::make_shared<rapidjson::Document>();
  doc->Parse<rapidjson::kParseIterativeFlag>(req.body.data(), req.body.size());
  if (doc->HasParseError()) {
    conn->WriteResponse(http_server::kStatusBadRequest,
                        JsonRpcFramingError(jsonrpc::kParseError, "parse error"), req.keep_alive);
    return;
  }
  if (!doc->IsObject() && !doc->IsArray()) {
    // Valid JSON, but not a JSON-RPC message at all: a framing-level 400.
    conn->WriteResponse(
        http_server::kStatusBadRequest,
        JsonRpcFramingError(jsonrpc::kInvalidRequest, "request must be a JSON object or array"),
        req.keep_alive);
    return;
  }

  const bool wants_sse = WantsSse(req);
  const Dispatcher* dispatcher = &dispatcher_;
  http_server::QueueWork(
      std::move(conn), loop_,
      [dispatcher, doc, wants_sse]() -> http_server::WorkResult {
        return BuildMcpWorkResult(*dispatcher, *doc, wants_sse);
      },
      req.keep_alive, inflight_);
}

}  // namespace bf::mcp
