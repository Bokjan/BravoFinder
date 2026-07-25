// SPDX-License-Identifier: MIT
// dispatcher.cc — the transport-neutral MCP method dispatch for bf-mcp.
//
// This is the protocol logic only: it takes a parsed JSON-RPC request, runs the
// three MCP methods (initialize / tools/list / tools/call) plus the
// server-provided list_cycles tool, and returns the JSON-RPC response envelope
// as a string. It does no byte I/O -- the stdio and HTTP transports own that.
// The capabilities themselves live in tools.cc as Tool objects; this file
// iterates that list rather than knowing about any individual tool.

#include "dispatcher.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <optional>
#include <string>
#include <string_view>

#include "core/version.h"
#include "handlers.h"
#include "jsonrpc.h"
#include "render.h"  // bf::service::JsonError

namespace bf::mcp {

namespace {

// A server-provided tool (not a per-database capability): lists the AIRAC
// cycles the registry can serve. Handled directly by the Dispatcher since it
// needs the registry, not a single NavDatabase.
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

// Negotiate the MCP protocol version. Echo the client's requested version when
// it is one we support; otherwise fall back to the default (which keeps a client
// that sends no protocolVersion on the historical stdio behavior).
const char* NegotiateProtocolVersion(const rapidjson::Value* params) {
  if (params != nullptr && params->IsObject() && params->HasMember("protocolVersion") &&
      (*params)["protocolVersion"].IsString()) {
    const std::string_view requested = (*params)["protocolVersion"].GetString();
    if (requested == kProtocolVersion2025) {
      return kProtocolVersion2025;
    }
    if (requested == kProtocolVersion2024) {
      return kProtocolVersion2024;
    }
  }
  return kDefaultProtocolVersion;
}

}  // namespace

Dispatcher::Response Dispatcher::Dispatch(const rapidjson::Value& request) const {
  // A batch is a JSON array of requests. Handle it as one unit so both the
  // stdio and HTTP transports get batch support from this single entry point.
  if (request.IsArray()) {
    return DispatchBatch(request);
  }
  if (!request.IsObject()) {
    return {};  // not a JSON object: nothing to reply to
  }
  const bool has_id = request.HasMember("id");
  if (!request.HasMember("method") || !request["method"].IsString()) {
    // Invalid request: only reply if it carried an id (a notification with no
    // method is silently dropped, per JSON-RPC).
    if (has_id) {
      return {MakeError(request["id"], jsonrpc::kInvalidRequest,
                        "invalid request: missing or non-string method"),
              true};
    }
    return {};
  }
  const std::string method = request["method"].GetString();

  if (method == "initialize") {
    if (!has_id) {
      return {};  // notification: no response
    }
    const rapidjson::Value* params = request.HasMember("params") ? &request["params"] : nullptr;
    return {HandleInitialize(request["id"], params), true};
  }
  if (method == "tools/list") {
    if (!has_id) {
      return {};
    }
    return {HandleToolsList(request["id"]), true};
  }
  if (method == "tools/call") {
    if (!has_id) {
      return {};
    }
    // A tools/call request must carry a params object; without one it is a
    // protocol error rather than a tool error.
    if (!request.HasMember("params") || !request["params"].IsObject()) {
      return {
          MakeError(request["id"], jsonrpc::kInvalidParams, "tools/call requires a params object"),
          true};
    }
    return {HandleToolsCall(request["id"], request["params"]), true};
  }
  // Unknown method: reply with method-not-found only for a request (has id).
  if (has_id) {
    return {MakeError(request["id"], jsonrpc::kMethodNotFound, "method not found: " + method),
            true};
  }
  return {};
}

Dispatcher::Response Dispatcher::DispatchBatch(const rapidjson::Value& batch) const {
  // An empty batch is a formed-but-invalid JSON-RPC message: return a single
  // -32600 error envelope (id null), per JSON-RPC 2.0 -- not an array.
  if (batch.Empty()) {
    const rapidjson::Value null_id(rapidjson::kNullType);
    return {MakeError(null_id, jsonrpc::kInvalidRequest, "invalid request: empty batch"), true};
  }
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> w(buf);
  w.StartArray();
  bool has_response = false;
  for (const rapidjson::Value& item : batch.GetArray()) {
    if (!item.IsObject()) {
      // A non-object batch element is not a valid JSON-RPC request: emit a
      // -32600 error entry (id null) so the client sees the rejection instead
      // of a silently shortened response array, per JSON-RPC 2.0.
      has_response = true;
      const rapidjson::Value null_id(rapidjson::kNullType);
      const std::string err = MakeError(null_id, jsonrpc::kInvalidRequest, "invalid request");
      w.RawValue(err.c_str(), err.size(), rapidjson::kObjectType);
      continue;
    }
    const Response r = Dispatch(item);
    if (r.has_response) {
      has_response = true;
      // r.body is a complete JSON-RPC envelope; splice it in verbatim.
      w.RawValue(r.body.c_str(), r.body.size(), rapidjson::kObjectType);
    }
  }
  w.EndArray();
  if (!has_response) {
    return {};  // every element was a notification: no reply
  }
  return {buf.GetString(), true};
}

std::string Dispatcher::MakeResult(const rapidjson::Value& id, rapidjson::Value& result) const {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String(jsonrpc::kVersion);
  writer.Key("id");
  id.Accept(writer);  // echo the client's id verbatim (int/string/null)
  writer.Key("result");
  result.Accept(writer);
  writer.EndObject();
  return buffer.GetString();
}

std::string Dispatcher::MakeError(const rapidjson::Value& id, int code,
                                  const std::string& message) const {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String(jsonrpc::kVersion);
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
  return buffer.GetString();
}

std::string Dispatcher::MakeToolResult(const rapidjson::Value& id, const std::string& json_text,
                                       bool is_error, uint32_t elapsed_ms) const {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("jsonrpc");
  writer.String(jsonrpc::kVersion);
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
  return buffer.GetString();
}

std::string Dispatcher::HandleInitialize(const rapidjson::Value& id,
                                         const rapidjson::Value* params) const {
  rapidjson::Document result;
  result.SetObject();
  auto& alloc = result.GetAllocator();
  result.AddMember("protocolVersion", rapidjson::Value(NegotiateProtocolVersion(params), alloc),
                   alloc);
  rapidjson::Value server_info(rapidjson::kObjectType);
  server_info.AddMember("name", rapidjson::Value("bf-mcp", alloc), alloc);
  server_info.AddMember("version", rapidjson::Value(kBravoFinderVersion, alloc), alloc);
  result.AddMember("serverInfo", server_info, alloc);
  rapidjson::Value capabilities(rapidjson::kObjectType);
  rapidjson::Value tools(rapidjson::kObjectType);
  tools.AddMember("listChanged", false, alloc);
  capabilities.AddMember("tools", tools, alloc);
  result.AddMember("capabilities", capabilities, alloc);
  return MakeResult(id, result);
}

std::string Dispatcher::HandleToolsList(const rapidjson::Value& id) const {
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
  return MakeResult(id, result);
}

// Serialize the registry's available cycles as a JSON array, newest first.
std::string Dispatcher::HandleListCycles(const rapidjson::Value& id) const {
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
  // An empty cycle list is a valid result, not a tool error: report success.
  // (In practice the server fail-fasts on an empty inventory at startup, so the
  // registry is never empty here, but the flag must reflect semantics either
  // way rather than conflate "no cycles" with "the call failed".)
  return MakeToolResult(id, buffer.GetString(), /*is_error=*/false);
}

std::string Dispatcher::HandleToolsCall(const rapidjson::Value& id,
                                        const rapidjson::Value& params) const {
  if (!params.HasMember("name") || !params["name"].IsString()) {
    return MakeError(id, jsonrpc::kInvalidParams, "missing tool name");
  }
  const std::string name = params["name"].GetString();

  if (name == kListCyclesTool) {
    return HandleListCycles(id);
  }

  // A missing/non-object "arguments" is treated as empty: tools validate their
  // own required fields and report a tool error if needed. This must be an
  // empty object (not the default kNullType): the code below calls HasMember /
  // Is* / Get* on it, and rapidjson asserts IsObject() inside FindMember, so a
  // null value would abort() in a debug build (NDEBUG undefined).
  rapidjson::Value null_args(rapidjson::kObjectType);
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
      return MakeToolResult(
          id, bf::service::JsonError("cycle must be a non-negative integer (omit for latest)"),
          true);
    }
    cycle = args["cycle"].GetUint();
  }
  Result<const NavDatabase*> db = registry_.Get(cycle);
  if (!db) {
    return MakeToolResult(id, bf::service::JsonError(db.error().message), true);
  }

  for (const Tool& tool : tools_) {
    if (tool.name == name) {
      ToolResult tr = tool.handler(args, *db.value());
      return MakeToolResult(id, tr.json_text, tr.is_error, tr.elapsed_ms);
    }
  }
  // The tool name is client-controlled, so escape it (a name with a quote must
  // not break the JSON frame).
  return MakeToolResult(id, bf::service::JsonError("unknown tool: " + name), true);
}

}  // namespace bf::mcp
