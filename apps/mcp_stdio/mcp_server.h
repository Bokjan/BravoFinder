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
  explicit McpServer(NavDatabaseRegistry& registry) : registry_(registry) {}

  // Run the stdio request/response loop until stdin closes. Returns the process
  // exit status.
  int Run();

 private:
  // A persistent allocator host. Tool schemas are deep-copied into this
  // Document's allocator so each Tool owns its schema independently of whatever
  // temporary Document parsed the schema string.
  rapidjson::Document schema_store_;

  NavDatabaseRegistry& registry_;

  void HandleInitialize(int id);
  void HandleToolsList(int id);
  void HandleToolsCall(int id, const rapidjson::Value& params);
  void HandleListCycles(int id);

  // JSON-RPC response helpers.
  void SendResult(int id, rapidjson::Value& result);
  void SendError(int id, int code, const std::string& message);
  void SendToolResult(int id, const std::string& json_text, bool is_error);
};

}  // namespace bf::mcp
