// mcp_server.cc — the stdio JSON-RPC transport for bf-mcp-stdio.
//
// This file is the protocol layer only: it reads JSON-RPC requests from stdin,
// dispatches the three methods MCP needs (initialize / tools/list / tools/call),
// and writes JSON-RPC responses to stdout. The capabilities themselves are
// defined in tools.cc as Tool objects; this file iterates that list rather
// than knowing about any individual tool.

#include "mcp_server.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "core/version.h"
#include "tools.h"

namespace bf::mcp {

namespace {

constexpr char kProtocolVersion[] = "2024-11-05";

// Parse a JSON-RPC request line. Returns false if the line is not a valid JSON
// object (the caller then silently skips it, as a well-behaved client sends
// well-formed JSON).
bool ParseRequest(const std::string& line, rapidjson::Document& doc) {
  if (line.empty()) {
    return false;
  }
  return !doc.Parse(line.c_str()).HasParseError() && doc.IsObject();
}

}  // namespace

int McpServer::Run() {
  std::string line;
  while (std::getline(std::cin, line)) {
    rapidjson::Document doc;
    if (!ParseRequest(line, doc)) {
      continue;
    }
    if (!doc.HasMember("method") || !doc["method"].IsString()) {
      continue;
    }
    const std::string method = doc["method"].GetString();
    const int id = doc.HasMember("id") && doc["id"].IsInt() ? doc["id"].GetInt() : 0;

    if (method == "initialize") {
      HandleInitialize(id);
    } else if (method == "tools/list") {
      HandleToolsList(id);
    } else if (method == "tools/call") {
      // A tools/call request must carry a params object; without one it is a
      // protocol error rather than a tool error.
      if (!doc.HasMember("params") || !doc["params"].IsObject()) {
        SendError(id, -32602, "tools/call requires a params object");
        continue;
      }
      HandleToolsCall(id, doc["params"]);
    }
    // Notifications (no id) and unknown methods are silently ignored.
  }
  return 0;
}

void McpServer::SendResult(int id, rapidjson::Value& result) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String("2.0");
  writer.Key("id");
  writer.Int(id);
  writer.Key("result");
  result.Accept(writer);
  writer.EndObject();
  std::cout << buffer.GetString() << "\n";
  std::cout.flush();
}

void McpServer::SendError(int id, int code, const std::string& message) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String("2.0");
  writer.Key("id");
  writer.Int(id);
  writer.Key("error");
  writer.StartObject();
  writer.Key("code");
  writer.Int(code);
  writer.Key("message");
  writer.String(message.c_str(), static_cast<unsigned>(message.size()));
  writer.EndObject();
  writer.EndObject();
  std::cout << buffer.GetString() << "\n";
  std::cout.flush();
}

void McpServer::SendToolResult(int id, const std::string& json_text, bool is_error) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String("2.0");
  writer.Key("id");
  writer.Int(id);
  writer.Key("result");
  writer.StartObject();
  writer.Key("content");
  writer.StartArray();
  writer.StartObject();
  writer.Key("type");
  writer.String("text");
  writer.Key("text");
  writer.String(json_text.c_str(), static_cast<unsigned>(json_text.size()));
  writer.EndObject();
  writer.EndArray();
  writer.Key("isError");
  writer.Bool(is_error);
  writer.EndObject();
  writer.EndObject();
  std::cout << buffer.GetString() << "\n";
  std::cout.flush();
}

void McpServer::HandleInitialize(int id) {
  rapidjson::Document result;
  result.SetObject();
  auto& alloc = result.GetAllocator();
  result.AddMember("protocolVersion", rapidjson::Value(kProtocolVersion, alloc), alloc);
  rapidjson::Value server_info(rapidjson::kObjectType);
  server_info.AddMember("name", rapidjson::Value("bf-mcp-stdio", alloc), alloc);
  server_info.AddMember("version", rapidjson::Value(kBravoFinderVersion, alloc), alloc);
  result.AddMember("serverInfo", server_info, alloc);
  rapidjson::Value capabilities(rapidjson::kObjectType);
  rapidjson::Value tools(rapidjson::kObjectType);
  tools.AddMember("listChanged", false, alloc);
  capabilities.AddMember("tools", tools, alloc);
  result.AddMember("capabilities", capabilities, alloc);
  SendResult(id, result);
}

void McpServer::HandleToolsList(int id) {
  rapidjson::Document result;
  result.SetObject();
  auto& alloc = result.GetAllocator();
  rapidjson::Value tools;
  tools.SetArray();

  for (const Tool& tool : AllTools()) {
    rapidjson::Value entry;
    entry.SetObject();
    entry.AddMember("name", rapidjson::Value(tool.name.c_str(), alloc), alloc);
    entry.AddMember("description", rapidjson::Value(tool.description.c_str(), alloc), alloc);
    // Deep-copy the tool's schema into this response's allocator so the tool's
    // own (server-owned) allocator is not relied upon past this call.
    rapidjson::Value schema_copy;
    schema_copy.CopyFrom(tool.input_schema, alloc);
    entry.AddMember("inputSchema", schema_copy.Move(), alloc);
    tools.PushBack(entry.Move(), alloc);
  }

  result.AddMember("tools", tools, alloc);
  SendResult(id, result);
}

void McpServer::HandleToolsCall(int id, const rapidjson::Value& params) {
  if (!params.HasMember("name") || !params["name"].IsString()) {
    SendError(id, -32602, "missing tool name");
    return;
  }
  const std::string name = params["name"].GetString();
  // A missing/non-object "arguments" is treated as empty: tools validate their
  // own required fields and report a tool error if needed.
  rapidjson::Value null_args;
  const rapidjson::Value& args = params.HasMember("arguments") && params["arguments"].IsObject()
                                     ? params["arguments"]
                                     : null_args;

  for (const Tool& tool : AllTools()) {
    if (tool.name == name) {
      auto [json, is_error] = tool.handler(args, db_);
      SendToolResult(id, json, is_error);
      return;
    }
  }
  SendToolResult(id, R"({"error":"unknown tool: )" + name + R"("})", true);
}

}  // namespace bf::mcp
