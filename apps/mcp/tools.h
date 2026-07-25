// tools.h — the MCP tool abstraction for bf-mcp.
//
// A Tool is fully self-describing: it carries its MCP name, a human/LLM-facing
// description, its JSON-Schema input descriptor, and the handler that implements
// it. The handler logic itself lives in bf::service (libs/service/), shared
// with the HTTP transport; MakeTools() attaches the MCP-specific description and
// JSON-Schema to each shared handler by name and adapts its HandlerResult to the
// {json_text, is_error} shape the stdio server writes. The server owns the tool
// vector as a member, iterating it for tools/list and looking up by name for
// tools/call.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "io/nav_database.h"
#include "rapidjson/document.h"

namespace bf::mcp {

// The result of running a tool: a JSON value (object or array) that the server
// wraps in a single text content block, the MCP error projection of the shared
// handler's status (status >= 400), and the database call's compute cost in
// milliseconds (0 on any error path, mirroring HandlerResult::elapsed_ms).
struct ToolResult {
  std::string json_text;
  bool is_error;
  uint32_t elapsed_ms = 0;
};

// A tool handler: given the parsed "arguments" object and a read-only database,
// returns a ToolResult.
using ToolHandler =
    std::function<ToolResult(const rapidjson::Value& args, const bf::NavDatabase& db)>;

struct Tool {
  std::string name;
  std::string description;
  // The inputSchema object for MCP self-description. RapidJSON Values borrow the
  // allocator of the Document that parsed them, so the Tool owns that Document
  // (schema_store_) to keep the allocator alive for as long as the schema lives.
  // input_schema is a view into schema_store_ and must be initialized after it.
  rapidjson::Document schema_store;
  rapidjson::Value input_schema;
  ToolHandler handler;

  // Construct a Tool from a parsed schema Document. schema_store takes ownership
  // of the Document (and thus the allocator); input_schema is then copied into
  // that same allocator so the two stay coupled for the Tool's lifetime.
  Tool(std::string tool_name, std::string tool_description, rapidjson::Document schema,
       ToolHandler tool_handler);
};

// Build every tool the server exposes, in display order. Takes the shared
// bf::service handlers and dresses each with its MCP description + JSON-Schema.
// Returns a vector (moved, since Tool holds a non-copyable rapidjson::Document)
// for the server to own as a member -- no function-level static mutable state.
std::vector<Tool> MakeTools();

}  // namespace bf::mcp
