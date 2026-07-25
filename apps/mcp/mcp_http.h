// SPDX-License-Identifier: MIT
// mcp_http.h — the MCP-over-HTTP (Streamable HTTP, 2025-03-26) transport for
// bf-mcp.
//
// McpHttpHandler is the RequestHandler for the shared HTTP core (libs/http_server/),
// serving the single MCP endpoint /mcp. It reuses the transport-neutral
// Dispatcher (the same one the stdio transport uses) for all protocol decisions,
// and offloads the 10-30 ms tools/call compute to the libuv threadpool exactly
// like the REST server -- the response write (inline or SSE) always happens back
// on the loop thread.
//
// The transport is stateless-with-id: initialize returns a random Mcp-Session-Id
// that the server does not track (every request is independent, since the
// Dispatcher holds no per-session state). POST /mcp answers JSON-RPC requests
// (single or batch), GET /mcp opens an SSE stream (no server-push today; a
// placeholder for future notifications), and DELETE /mcp acknowledges a session
// teardown.

#pragma once

#include <uv.h>

#include <atomic>
#include <memory>

#include "dispatcher.h"  // bf::mcp::Dispatcher
#include "registry.h"
#include "transport.h"  // http_server::RequestHandler / HttpRequest / Connection (fwd)

namespace bf::mcp {

class McpHttpHandler : public http_server::RequestHandler {
 public:
  // Serve MCP from `registry`; offload work onto `loop`'s threadpool. Both must
  // outlive the handler. Builds its Dispatcher (and tool list) once.
  McpHttpHandler(bf::service::NavDatabaseRegistry& registry, uv_loop_t* loop)
      : dispatcher_(registry), loop_(loop) {}

  // Route one fully-parsed request (loop thread): answer /mcp per Streamable
  // HTTP, or 404 anything else.
  void Handle(std::shared_ptr<http_server::Connection> conn,
              const http_server::HttpRequest& req) override;

 private:
  // Handle POST /mcp: parse the JSON-RPC body (single or batch) and either
  // answer inline (framing errors, pure notifications) or offload the dispatch.
  void HandlePost(std::shared_ptr<http_server::Connection> conn,
                  const http_server::HttpRequest& req);

  // The protocol core, shared with the stdio transport. Dispatch is const and
  // thread-safe, so many worker threads may call it concurrently.
  Dispatcher dispatcher_;
  uv_loop_t* loop_;
  // In-flight offloaded work items; a request over the cap is shed with 503.
  std::atomic<int> inflight_{0};
};

}  // namespace bf::mcp
