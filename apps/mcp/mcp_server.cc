// mcp_server.cc — the stdio JSON-RPC transport for bf-mcp.
//
// This file is the protocol layer only: it reads JSON-RPC requests from stdin,
// dispatches the three methods MCP needs (initialize / tools/list / tools/call),
// and writes JSON-RPC responses to stdout. The capabilities themselves are
// defined in tools.cc as Tool objects; this file iterates that list rather
// than knowing about any individual tool.

#include "mcp_server.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <optional>
#include <string>

#include "core/version.h"
#include "handlers.h"
#include "tools.h"

namespace bf::mcp {

namespace {

constexpr char kProtocolVersion[] = "2024-11-05";

// A server-provided tool (not a per-database capability): lists the AIRAC
// cycles the registry can serve. Handled directly by McpServer since it needs
// the registry, not a single NavDatabase.
constexpr char kListCyclesTool[] = "list_cycles";

// Inject the shared "cycle" argument into a per-database tool's schema. cycle
// is a server-level concern (which database to query), so it is added here
// rather than duplicated into every tool's own schema string.
void InjectCycleProperty(rapidjson::Value& schema, rapidjson::Document::AllocatorType& alloc) {
  if (!schema.IsObject() || !schema.HasMember("properties") || !schema["properties"].IsObject()) {
    return;
  }
  rapidjson::Value cycle(rapidjson::kObjectType);
  cycle.AddMember("type", "integer", alloc);
  cycle.AddMember(
      "description",
      rapidjson::Value("AIRAC cycle to query, e.g. 2601. Omit to use the latest loaded cycle. "
                       "Use list_cycles to see what is available.",
                       alloc),
      alloc);
  schema["properties"].AddMember("cycle", cycle, alloc);
}

// Parse a JSON-RPC request line. Returns false if the line is not a valid JSON
// object (the caller then silently skips it, as a well-behaved client sends
// well-formed JSON).
bool ParseRequest(const std::string& line, rapidjson::Document& doc) {
  if (line.empty()) {
    return false;
  }
  // Parse iteratively (kParseIterativeFlag) over (data, size): the line is
  // untrusted, and rapidjson's default recursive-descent parser would blow the
  // C++ stack on a deeply nested payload. Using the length form also stops an
  // embedded NUL from truncating the request.
  return !doc.Parse<rapidjson::kParseIterativeFlag>(line.data(), line.size()).HasParseError() &&
         doc.IsObject();
}

// Read one newline-terminated line from `in`, bounding memory to `max_len`
// bytes. std::getline grows the target string without limit, so a client that
// streams bytes and never sends a newline (exactly the attack the cap is meant
// to stop) makes getline consume until OOM -- the size check afterward never
// runs. This reader stops growing `out` once `max_len` is reached and keeps
// draining the rest of the oversized line to the newline, so the buffer never
// exceeds the cap. kEof is returned only at end of input with no pending bytes.
enum class LineResult { kOk, kOversized, kEof };

LineResult ReadBoundedLine(std::istream& in, std::string& out, size_t max_len) {
  out.clear();
  bool oversized = false;
  bool saw_any = false;
  char ch = 0;
  while (in.get(ch)) {
    saw_any = true;
    if (ch == '\n') {
      return oversized ? LineResult::kOversized : LineResult::kOk;
    }
    if (out.size() < max_len) {
      out.push_back(ch);
    } else {
      oversized = true;  // keep draining to the newline without growing `out`
    }
  }
  if (!saw_any) {
    return LineResult::kEof;  // clean end of input
  }
  return oversized ? LineResult::kOversized : LineResult::kOk;  // final unterminated line
}

}  // namespace

int McpServer::Run() {
  // JSON-RPC over stdio: one request per line, one response per line on stdout.
  // A request with no "id" is a notification (per JSON-RPC 2.0) and gets no
  // response. A request with an id (int, string, or null) is echoed back verbatim
  // so the client can match the response.
  std::string line;
  // Cap a single request line: a malicious or buggy client could stream without
  // a newline and grow `line` until OOM. The MCP frame is bounded by realistic
  // tool arguments; anything larger is drained and rejected without buffering it
  // whole (see ReadBoundedLine).
  constexpr size_t kMaxLineLen = 16 * 1024 * 1024;  // 16 MiB
  for (;;) {
    const LineResult lr = ReadBoundedLine(std::cin, line, kMaxLineLen);
    if (lr == LineResult::kEof) {
      break;
    }
    if (lr == LineResult::kOversized) {
      // The oversized line was drained to its newline; drop it (there is no
      // request id to reply to yet).
      continue;
    }
    rapidjson::Document doc;
    if (!ParseRequest(line, doc)) {
      continue;
    }
    if (!doc.HasMember("method") || !doc["method"].IsString()) {
      // Invalid request: only reply if it carried an id (a notification with no
      // method is silently dropped, per JSON-RPC).
      if (doc.HasMember("id")) {
        SendError(doc["id"], -32600, "invalid request: missing or non-string method");
      }
      continue;
    }
    const std::string method = doc["method"].GetString();
    const bool has_id = doc.HasMember("id");

    if (method == "initialize") {
      if (!has_id) {
        continue;  // notification: no response
      }
      HandleInitialize(doc["id"]);
    } else if (method == "tools/list") {
      if (!has_id) {
        continue;
      }
      HandleToolsList(doc["id"]);
    } else if (method == "tools/call") {
      if (!has_id) {
        continue;
      }
      // A tools/call request must carry a params object; without one it is a
      // protocol error rather than a tool error.
      if (!doc.HasMember("params") || !doc["params"].IsObject()) {
        SendError(doc["id"], -32602, "tools/call requires a params object");
        continue;
      }
      HandleToolsCall(doc["id"], doc["params"]);
    } else {
      // Unknown method: reply with method-not-found only for a request (has id).
      if (has_id) {
        SendError(doc["id"], -32601, "method not found: " + method);
      }
    }
  }
  return 0;
}

void McpServer::SendResult(const rapidjson::Value& id, rapidjson::Value& result) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String("2.0");
  writer.Key("id");
  id.Accept(writer);  // echo the client's id verbatim (int/string/null)
  writer.Key("result");
  result.Accept(writer);
  writer.EndObject();
  std::cout << buffer.GetString() << "\n";
  std::cout.flush();
}

void McpServer::SendError(const rapidjson::Value& id, int code, const std::string& message) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String("2.0");
  writer.Key("id");
  id.Accept(writer);  // echo the client's id verbatim (int/string/null)
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

void McpServer::SendToolResult(const rapidjson::Value& id, const std::string& json_text,
                               bool is_error, uint32_t elapsed_ms) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String("2.0");
  writer.Key("id");
  id.Accept(writer);  // echo the client's id verbatim (int/string/null)
  writer.Key("result");
  writer.StartObject();
  writer.Key("content");
  writer.StartArray();
  writer.StartObject();
  writer.Key("type");
  writer.String("text");
  writer.Key("text");
  // json_text is already a serialized JSON value; embed it as a string (the
  // Writer escapes it), preserving its structure for the client to parse.
  writer.String(json_text.c_str(), static_cast<unsigned>(json_text.size()));
  writer.EndObject();
  writer.EndArray();
  writer.Key("isError");
  writer.Bool(is_error);
  // Surface the database call's compute cost as MCP result metadata. Only a
  // successful tool call carries a non-zero timing (error paths leave it 0), so
  // omit the field entirely otherwise rather than reporting a misleading 0.
  if (elapsed_ms > 0) {
    writer.Key("_meta");
    writer.StartObject();
    writer.Key("elapsed_ms");
    writer.Uint(elapsed_ms);
    writer.EndObject();
  }
  writer.EndObject();
  writer.EndObject();
  std::cout << buffer.GetString() << "\n";
  std::cout.flush();
}

void McpServer::HandleInitialize(const rapidjson::Value& id) {
  rapidjson::Document result;
  result.SetObject();
  auto& alloc = result.GetAllocator();
  result.AddMember("protocolVersion", rapidjson::Value(kProtocolVersion, alloc), alloc);
  rapidjson::Value server_info(rapidjson::kObjectType);
  server_info.AddMember("name", rapidjson::Value("bf-mcp", alloc), alloc);
  server_info.AddMember("version", rapidjson::Value(kBravoFinderVersion, alloc), alloc);
  result.AddMember("serverInfo", server_info, alloc);
  rapidjson::Value capabilities(rapidjson::kObjectType);
  rapidjson::Value tools(rapidjson::kObjectType);
  tools.AddMember("listChanged", false, alloc);
  capabilities.AddMember("tools", tools, alloc);
  result.AddMember("capabilities", capabilities, alloc);
  SendResult(id, result);
}

void McpServer::HandleToolsList(const rapidjson::Value& id) {
  rapidjson::Document result;
  result.SetObject();
  auto& alloc = result.GetAllocator();
  rapidjson::Value tools;
  tools.SetArray();

  for (const Tool& tool : tools_) {
    rapidjson::Value entry;
    entry.SetObject();
    entry.AddMember("name", rapidjson::Value(tool.name.c_str(), alloc), alloc);
    entry.AddMember("description", rapidjson::Value(tool.description.c_str(), alloc), alloc);
    // Deep-copy the tool's schema into this response's allocator so the tool's
    // own (server-owned) allocator is not relied upon past this call.
    rapidjson::Value schema_copy;
    schema_copy.CopyFrom(tool.input_schema, alloc);
    // Every per-database tool accepts an optional "cycle"; inject it here so the
    // tools stay cycle-agnostic and the argument is declared in one place.
    InjectCycleProperty(schema_copy, alloc);
    entry.AddMember("inputSchema", schema_copy.Move(), alloc);
    tools.PushBack(entry.Move(), alloc);
  }

  // The server-provided list_cycles tool (no cycle argument of its own).
  {
    rapidjson::Value entry(rapidjson::kObjectType);
    entry.AddMember("name", rapidjson::Value(kListCyclesTool, alloc), alloc);
    entry.AddMember(
        "description",
        rapidjson::Value("List the AIRAC cycles this server can query. Returns each cycle and "
                         "whether it is already loaded. Pass a cycle to the other tools' "
                         "'cycle' argument to query a specific one.",
                         alloc),
        alloc);
    rapidjson::Value schema(rapidjson::kObjectType);
    schema.AddMember("type", "object", alloc);
    schema.AddMember("properties", rapidjson::Value(rapidjson::kObjectType), alloc);
    entry.AddMember("inputSchema", schema, alloc);
    tools.PushBack(entry.Move(), alloc);
  }

  result.AddMember("tools", tools, alloc);
  SendResult(id, result);
}

// Serialize the registry's available cycles as a JSON array, newest first.
void McpServer::HandleListCycles(const rapidjson::Value& id) {
  const BfdbInventory& inv = registry_.inventory();
  // A cycle is "loaded" once Get has opened it; the inventory does not track
  // that, so we only report the cycle here (loaded state is transient and not
  // essential for the client's choice).
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartArray();
  // Entries are sorted ascending; present newest first for readability.
  const auto& entries = inv.entries();
  for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
    writer.StartObject();
    writer.Key("cycle");
    writer.Uint(it->cycle);
    writer.EndObject();
  }
  writer.EndArray();
  SendToolResult(id, buffer.GetString(), inv.empty());
}

void McpServer::HandleToolsCall(const rapidjson::Value& id, const rapidjson::Value& params) {
  if (!params.HasMember("name") || !params["name"].IsString()) {
    SendError(id, -32602, "missing tool name");
    return;
  }
  const std::string name = params["name"].GetString();

  if (name == kListCyclesTool) {
    HandleListCycles(id);
    return;
  }

  // A missing/non-object "arguments" is treated as empty: tools validate their
  // own required fields and report a tool error if needed.
  rapidjson::Value null_args;
  const rapidjson::Value& args = params.HasMember("arguments") && params["arguments"].IsObject()
                                     ? params["arguments"]
                                     : null_args;

  // Resolve which database to serve from the optional "cycle" argument (absent
  // => latest). This is the server's concern, so the tool handlers never see
  // cycle and keep operating on a single database.
  std::optional<uint32_t> cycle;
  if (args.HasMember("cycle")) {
    // Reject a present-but-invalid cycle rather than silently falling back to
    // the latest: a client passing cycle:-1 would otherwise be served a
    // different cycle's data with no signal.
    if (!args["cycle"].IsUint()) {
      SendToolResult(
          id, bf::service::JsonError("cycle must be a non-negative integer (omit for latest)"),
          true);
      return;
    }
    cycle = args["cycle"].GetUint();
  }
  Result<const NavDatabase*> db = registry_.Get(cycle);
  if (!db) {
    SendToolResult(id, bf::service::JsonError(db.error().message), true);
    return;
  }

  for (const Tool& tool : tools_) {
    if (tool.name == name) {
      ToolResult tr = tool.handler(args, *db.value());
      SendToolResult(id, tr.json_text, tr.is_error, tr.elapsed_ms);
      return;
    }
  }
  // The tool name is client-controlled, so escape it (a name with a quote must
  // not break the JSON frame).
  SendToolResult(id, bf::service::JsonError("unknown tool: " + name), true);
}

}  // namespace bf::mcp
