// SPDX-License-Identifier: MIT
// stdio_runner.cc — the stdio JSON-RPC transport for bf-mcp.
//
// This file is the transport layer only: it reads JSON-RPC requests from stdin
// one line at a time (bounding memory against a client that never sends a
// newline), parses each into a Document, hands it to the Dispatcher, and writes
// the returned envelope to stdout. Every protocol decision -- which methods
// exist, how tools are called, the JSON-RPC framing -- lives in the Dispatcher;
// this file knows only about lines and streams.

#include "stdio_runner.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <iostream>
#include <string>
#include <string_view>

#include "jsonrpc.h"
#include "rapidjson/document.h"

namespace bf::mcp {

namespace {

// A JSON-RPC error envelope with a null id, for a framing failure where no
// request id can be extracted (oversized line, parse error, or a body that is
// neither a JSON object nor an array). Mirrors mcp_http's JsonRpcFramingError.
std::string JsonRpcFramingError(int code, const std::string& message) {
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> w(buf);
  w.StartObject();
  w.Key("jsonrpc");
  w.String(jsonrpc::kVersion);
  w.Key("id");
  w.Null();
  w.Key("error");
  w.StartObject();
  w.Key("code");
  w.Int(code);
  w.Key("message");
  w.String(message.c_str(), static_cast<unsigned>(message.size()));
  w.EndObject();
  w.EndObject();
  return buf.GetString();
}

void WriteStdoutLine(std::string_view payload) {
  std::cout << payload << '\n';
  std::cout.flush();
}

void WriteFramingError(int code, const std::string& message) {
  WriteStdoutLine(JsonRpcFramingError(code, message));
}

// Parse a JSON-RPC request line. Returns false if the line is not a valid JSON
// object or array (the caller then emits a framing error). An array is a
// JSON-RPC batch; the Dispatcher handles it as one unit, so the stdio transport
// gets batch support for free. On failure, *out_code / *out_message name the
// framing error to write (-32700 parse, -32600 invalid request).
bool ParseRequest(const std::string& line, rapidjson::Document& doc, int* out_code,
                  std::string* out_message) {
  if (line.empty()) {
    *out_code = jsonrpc::kParseError;
    *out_message = "empty request";
    return false;
  }
  // Parse iteratively (kParseIterativeFlag) over (data, size): the line is
  // untrusted, and rapidjson's default recursive-descent parser would blow the
  // C++ stack on a deeply nested payload. Using the length form also stops an
  // embedded NUL from truncating the request.
  if (doc.Parse<rapidjson::kParseIterativeFlag>(line.data(), line.size()).HasParseError()) {
    *out_code = jsonrpc::kParseError;
    *out_message = "parse error";
    return false;
  }
  if (!doc.IsObject() && !doc.IsArray()) {
    *out_code = jsonrpc::kInvalidRequest;
    *out_message = "request must be a JSON object or array";
    return false;
  }
  return true;
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
      // Strip a CRLF client's trailing '\r' so the caller sees the same line a
      // '\n'-only client would send. rapidjson tolerates trailing whitespace, so
      // this is defensive today, but keeps framing exact if that ever tightens.
      if (!out.empty() && out.back() == '\r') {
        out.pop_back();
      }
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

int StdioRunner::Run() {
  // JSON-RPC over stdio: one request per line, one response per line on stdout.
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
      // The oversized line was drained to its newline; there is no request id to
      // reply to, so emit a framing-level parse error (id null) and continue.
      WriteFramingError(jsonrpc::kParseError, "request line too large");
      continue;
    }
    rapidjson::Document doc;
    int err_code = 0;
    std::string err_message;
    if (!ParseRequest(line, doc, &err_code, &err_message)) {
      WriteFramingError(err_code, err_message);
      continue;
    }
    const Dispatcher::Response resp = dispatcher_.Dispatch(doc);
    if (resp.has_response) {
      WriteStdoutLine(resp.body);
    }
  }
  return 0;
}

}  // namespace bf::mcp
