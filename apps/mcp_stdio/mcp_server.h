#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "io/nav_database.h"
#include "rapidjson/document.h"
#include "registry.h"
#include "tools.h"

namespace bf::mcp {

// McpServer speaks MCP over stdio as JSON-RPC 2.0. It owns the loop that reads
// one JSON-RPC request per line from stdin and writes one JSON-RPC response per
// line to stdout, and a NavDatabaseRegistry that serves one or more AIRAC
// cycles. Protocol details (framing, the methods) live here; the actual
// capabilities live in tools.cc as a list of Tool objects.
class McpServer {
 public:
  // Takes the registry to serve from. The registry must outlive the server.
  // Each database it holds is read-only per NavDatabase contract B.
  explicit McpServer(NavDatabaseRegistry& registry) : registry_(registry), tools_(MakeTools()) {}

  // Run the stdio request/response loop until stdin closes. Returns the process
  // exit status.
  int Run();

 private:
  // The registry to serve from. Each database it holds is read-only per
  // NavDatabase contract B.
  NavDatabaseRegistry& registry_;

  // The tool list, built once in the constructor (no function-level static
  // mutable state). Each Tool owns its rapidjson::Document schema, so the vector
  // holds move-only, non-copyable elements.
  std::vector<Tool> tools_;

  void HandleInitialize(const rapidjson::Value& id);
  void HandleToolsList(const rapidjson::Value& id);
  void HandleToolsCall(const rapidjson::Value& id, const rapidjson::Value& params);
  void HandleListCycles(const rapidjson::Value& id);

  // JSON-RPC response helpers. `id` is written back verbatim (int, string, or
  // null) so a client's request id round-trips unchanged; notifications (no id)
  // never reach these because Run() does not call them.
  void SendResult(const rapidjson::Value& id, rapidjson::Value& result);
  void SendError(const rapidjson::Value& id, int code, const std::string& message);
  void SendToolResult(const rapidjson::Value& id, const std::string& json_text, bool is_error);
};

}  // namespace bf::mcp
