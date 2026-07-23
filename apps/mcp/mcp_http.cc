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
  std::random_device rd;
  const auto word = [&rd]() -> uint64_t {
    return (static_cast<uint64_t>(rd()) << 32) | static_cast<uint64_t>(rd());
  };
  const uint64_t hi = word();
  const uint64_t lo = word();
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx", static_cast<unsigned long long>(hi),
                static_cast<unsigned long long>(lo));
  return std::string(buf, 32);
}

// Whether the request's Accept header opts into an SSE (text/event-stream)
// response. When present, the server replies with a single buffered SSE event;
// otherwise it replies with an application/json body.
bool WantsSse(const http_server::HttpRequest& req) {
  return req.Header("accept").find("text/event-stream") != std::string::npos;
}

// Run the dispatch(es) for one POST body and shape the result into a WorkResult.
// Runs on a worker thread. A single object dispatches once; an array dispatches
// each element and collects the non-notification responses into a JSON array. A
// batch element that is not a JSON object is an invalid request: it gets a
// -32600 (id null) entry in the response array rather than being silently
// dropped, per JSON-RPC 2.0. A body of only notifications (no response) becomes
// 202 with an empty body, per Streamable HTTP. Otherwise the response is framed
// as JSON or a single SSE event, with a fresh Mcp-Session-Id header when the
// request was initialize (a single object, or a batch containing one).
http_server::WorkResult BuildMcpWorkResult(const Dispatcher& dispatcher,
                                           const rapidjson::Document& req, bool wants_sse) {
  std::string json;
  bool has_response = false;
  bool is_initialize = false;

  if (req.IsArray()) {
    rapidjson::StringBuffer buf;
    rapidjson::Writer<rapidjson::StringBuffer> w(buf);
    w.StartArray();
    for (const rapidjson::Value& item : req.GetArray()) {
      if (!item.IsObject()) {
        // A non-object batch element is not a valid JSON-RPC request: emit a
        // -32600 error entry (id null) so the client sees the rejection instead
        // of a silently shortened response array.
        has_response = true;
        const std::string err = JsonRpcFramingError(jsonrpc::kInvalidRequest, "invalid request");
        w.RawValue(err.c_str(), err.size(), rapidjson::kObjectType);
        continue;
      }
      if (!is_initialize && item.HasMember("method") && item["method"].IsString() &&
          std::strcmp(item["method"].GetString(), "initialize") == 0) {
        is_initialize = true;
      }
      const Dispatcher::Response r = dispatcher.Dispatch(item);
      if (r.has_response) {
        has_response = true;
        // r.body is a complete JSON-RPC envelope; splice it in verbatim.
        w.RawValue(r.body.c_str(), r.body.size(), rapidjson::kObjectType);
      }
    }
    w.EndArray();
    json = buf.GetString();
  } else {
    const Dispatcher::Response r = dispatcher.Dispatch(req);
    has_response = r.has_response;
    json = r.body;
    if (req.IsObject() && req.HasMember("method") && req["method"].IsString()) {
      is_initialize = std::strcmp(req["method"].GetString(), "initialize") == 0;
    }
  }

  if (!has_response) {
    // Pure notification(s): acknowledge with 202 and no body.
    http_server::WorkResult out;
    out.status = 202;
    return out;
  }

  http_server::WorkResult out;
  out.status = 200;
  if (wants_sse) {
    out.content_type = "text/event-stream";
    out.body = "data: " + json + "\n\n";
  } else {
    out.content_type = "application/json";
    out.body = std::move(json);
  }
  if (is_initialize) {
    out.extra_headers.emplace_back("Mcp-Session-Id", GenerateSessionId());
  }
  return out;
}

}  // namespace

void McpHttpHandler::Handle(std::shared_ptr<http_server::Connection> conn,
                            const http_server::HttpRequest& req) {
  if (req.path != "/mcp") {
    conn->WriteResponse(404, http_server::JsonError("not found"), req.keep_alive);
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
    conn->BeginStream(200, "text/event-stream", {});
    conn->WriteEvent(": keepalive\n\n");
    return;
  }
  if (req.method == "DELETE") {
    // Stateless: acknowledge the session teardown without tracking anything.
    conn->WriteResponse(200, "", req.keep_alive);
    return;
  }
  conn->WriteResponse(405, http_server::JsonError("method not allowed"), req.keep_alive);
}

void McpHttpHandler::HandlePost(std::shared_ptr<http_server::Connection> conn,
                                const http_server::HttpRequest& req) {
  // Shed load before parsing if the offload queue is full.
  if (inflight_.load(std::memory_order_relaxed) >= kMaxInflightWork) {
    conn->WriteResponse(503, http_server::JsonError("server busy"), req.keep_alive);
    return;
  }
  if (req.body.empty()) {
    conn->WriteResponse(400, JsonRpcFramingError(jsonrpc::kParseError, "empty request body"),
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
    conn->WriteResponse(400, JsonRpcFramingError(jsonrpc::kParseError, "parse error"),
                        req.keep_alive);
    return;
  }
  if (!doc->IsObject() && !doc->IsArray()) {
    // Valid JSON, but not a JSON-RPC message at all: a framing-level 400.
    conn->WriteResponse(
        400,
        JsonRpcFramingError(jsonrpc::kInvalidRequest, "request must be a JSON object or array"),
        req.keep_alive);
    return;
  }
  if (doc->IsArray() && doc->Empty()) {
    // An empty batch is a formed JSON-RPC message (just empty), so it is a
    // JSON-RPC-level error rather than a transport framing failure: answer 200
    // with a single -32600 error envelope (id null), per JSON-RPC 2.0.
    conn->WriteResponse(
        200, JsonRpcFramingError(jsonrpc::kInvalidRequest, "invalid request: empty batch"),
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
