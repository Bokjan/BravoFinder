#pragma once

#include <iostream>
#include <memory>
#include <string>

#include "io/nav_database.h"
#include "rapidjson/document.h"
#include "tools.h"

namespace bf::mcp {

// McpServer speaks MCP over stdio as JSON-RPC 2.0. It owns the read-only
// NavDatabase and the loop that reads one JSON-RPC request per line from stdin
// and writes one JSON-RPC response per line to stdout. Protocol details
// (framing, the three methods) live here; the actual capabilities live in
// tools.cc as a list of Tool objects.
class McpServer {
 public:
  // Takes a database to serve from. The database must outlive the server and is
  // treated as read-only (per NavDatabase contract B).
  explicit McpServer(const NavDatabase& db) : db_(db) {}

  // Run the stdio request/response loop until stdin closes. Returns the process
  // exit status.
  int Run();

 private:
  // A persistent allocator host. Tool schemas are deep-copied into this
  // Document's allocator so each Tool owns its schema independently of whatever
  // temporary Document parsed the schema string.
  rapidjson::Document schema_store_;

  const NavDatabase& db_;

  void HandleInitialize(int id);
  void HandleToolsList(int id);
  void HandleToolsCall(int id, const rapidjson::Value& params);

  // JSON-RPC response helpers.
  void SendResult(int id, rapidjson::Value& result);
  void SendError(int id, int code, const std::string& message);
  void SendToolResult(int id, const std::string& json_text, bool is_error);
};

}  // namespace bf::mcp
