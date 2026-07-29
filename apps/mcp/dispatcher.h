// SPDX-License-Identifier: MIT
// dispatcher.h — the transport-neutral MCP JSON-RPC dispatcher for bf-mcp.
//
// Dispatcher owns everything about MCP that does not depend on how bytes move:
// the tool list, the three methods (initialize / tools/list / tools/call), the
// server-provided list_cycles tool, protocol-version negotiation, and building
// the JSON-RPC response envelope. It takes a parsed request Value and returns a
// serialized response string; it does no I/O of its own. The stdio transport
// (stdio_runner) reads a line, parses it, calls Dispatch, and writes the result;
// the HTTP transport (mcp_http) offloads Dispatch to the threadpool and writes
// it back over a socket. Because Dispatch only reads the registry (const,
// contract B) and the tools (const after construction), it is safe to call
// concurrently on one Dispatcher from many worker threads.

#pragma once

#include <string>
#include <vector>

#include "rapidjson/document.h"
#include "registry.h"
#include "tools.h"

namespace bf::mcp {

// The MCP protocol versions this server understands. Negotiation echoes the
// client's requested version when it is one of these; otherwise (an unsupported
// or absent version) it falls back to kDefaultProtocolVersion, which aliases the
// highest version we speak (kProtocolVersion2025), as MCP requires the server to
// answer with its newest understood version.
inline constexpr char kProtocolVersion2024[] = "2024-11-05";
inline constexpr char kProtocolVersion2025[] = "2025-03-26";
inline constexpr const char* kDefaultProtocolVersion = kProtocolVersion2025;

class Dispatcher {
 public:
  // The outcome of dispatching one request. `has_response` is false for a
  // notification (a request with no id), which per JSON-RPC gets no reply; the
  // transport then writes nothing. `body` is a complete JSON-RPC response
  // envelope (result or error) when has_response is true.
  struct Response {
    std::string body;
    bool has_response = false;
  };

  // Serve tools from `registry`, which must outlive the Dispatcher. Each opened
  // database is read-only per NavDatabase contract B. Builds the tool list once
  // in the constructor (no function-level static mutable state).
  explicit Dispatcher(bf::service::NavDatabaseRegistry& registry)
      : registry_(registry), tools_(MakeTools()) {}

  // Dispatch one parsed JSON-RPC request: a single object, or a batch (a JSON
  // array of objects). A batch yields a JSON array of the non-notification
  // responses (or no response when every element is a notification); an empty
  // batch yields a single -32600 error envelope. Runs synchronously on the
  // calling thread: the transport decides whether that thread is the stdio loop
  // (inline) or a libuv worker (offloaded). Returns the response envelope, or a
  // no-response marker for a notification / malformed request that carries no id.
  Response Dispatch(const rapidjson::Value& request) const;

 private:
  // Handle a batch (JSON array): dispatch each element and collect the
  // non-notification responses into a JSON array. A non-object element becomes
  // a -32600 (id null) entry; an empty batch becomes a single -32600 envelope.
  Response DispatchBatch(const rapidjson::Value& batch) const;
  // The registry to serve from. Each database it holds is read-only per
  // NavDatabase contract B.
  bf::service::NavDatabaseRegistry& registry_;

  // The tool list, built once in the constructor. Each Tool owns its
  // rapidjson::Document schema, so the vector holds move-only elements. const
  // after construction, so concurrent Dispatch calls only read it.
  std::vector<Tool> tools_;

  // Method handlers: each builds and returns a complete JSON-RPC envelope string.
  std::string HandleInitialize(const rapidjson::Value& id, const rapidjson::Value* params) const;
  std::string HandleToolsList(const rapidjson::Value& id) const;
  std::string HandleToolsCall(const rapidjson::Value& id, const rapidjson::Value& params) const;
  std::string HandleListCycles(const rapidjson::Value& id) const;

  // JSON-RPC envelope builders. `id` is echoed back verbatim (int, string, or
  // null) so a client's request id round-trips unchanged.
  std::string MakeResult(const rapidjson::Value& id, rapidjson::Value& result) const;
  std::string MakeError(const rapidjson::Value& id, int code, const std::string& message) const;
  std::string MakeToolResult(const rapidjson::Value& id, const std::string& json_text,
                             bool is_error, uint32_t elapsed_ms = 0) const;
};

}  // namespace bf::mcp
