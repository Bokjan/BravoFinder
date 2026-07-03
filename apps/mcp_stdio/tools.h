// tools.h — the MCP tool abstraction for bf-mcp-stdio.
//
// A Tool is fully self-describing: it carries its MCP name, a human/LLM-facing
// description, its JSON-Schema input descriptor, and the handler that implements
// it. Handlers take the request arguments and a read-only NavDatabase, so they
// are pure with respect to server state (no globals) and easy to reason about.
// The set of available tools is exposed via AllTools(); the server iterates
// that list for tools/list and looks up by name for tools/call.

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

// Every tool the server exposes, in display order. Defined in tools.cc.
const std::vector<Tool>& AllTools();

}  // namespace bf::mcp
