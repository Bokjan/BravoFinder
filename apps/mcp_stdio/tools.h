// tools.h — the MCP tool abstraction for bf-mcp-stdio.
//
// A Tool is fully self-describing: it carries its MCP name, a human/LLM-facing
// description, its JSON-Schema input descriptor, and the handler that implements
// it. Handlers take the request arguments and a read-only NavDatabase, so they
// are pure with respect to server state (no globals) and easy to reason about.
// The set of available tools is built by MakeTools(); the server owns that
// vector as a member, iterating it for tools/list and looking up by name for
// tools/call.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include "io/nav_database.h"
#include "rapidjson/document.h"

namespace bf::mcp {

// A tool handler: given the parsed "arguments" object and a read-only database,
// returns {json_text, is_error}. json_text is a JSON value (object or array)
// that the server wraps in a single text content block.
using ToolHandler = std::function<std::pair<std::string, bool>(const rapidjson::Value& args,
                                                               const bf::NavDatabase& db)>;

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

// Build every tool the server exposes, in display order. Returns a vector
// (moved, since Tool holds a non-copyable rapidjson::Document) for the server
// to own as a member -- no function-level static mutable state.
std::vector<Tool> MakeTools();

// Build a tool-error JSON payload `{"error":"<message>"}` with RapidJSON's
// Writer so the message is auto-escaped. Tool error messages may carry
// user-controlled strings (an unknown departure airport, a bad route token, an
// unknown tool name); this is the only sanctioned way to emit such a payload.
std::string JsonError(const std::string& message);

}  // namespace bf::mcp
