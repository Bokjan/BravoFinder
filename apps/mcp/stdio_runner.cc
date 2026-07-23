// stdio_runner.cc — the stdio JSON-RPC transport for bf-mcp.
//
// This file is the transport layer only: it reads JSON-RPC requests from stdin
// one line at a time (bounding memory against a client that never sends a
// newline), parses each into a Document, hands it to the Dispatcher, and writes
// the returned envelope to stdout. Every protocol decision -- which methods
// exist, how tools are called, the JSON-RPC framing -- lives in the Dispatcher;
// this file knows only about lines and streams.

#include "stdio_runner.h"

#include <iostream>
#include <string>

#include "rapidjson/document.h"

namespace bf::mcp {

namespace {

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
      // The oversized line was drained to its newline; drop it (there is no
      // request id to reply to yet).
      continue;
    }
    rapidjson::Document doc;
    if (!ParseRequest(line, doc)) {
      continue;
    }
    const Dispatcher::Response resp = dispatcher_.Dispatch(doc);
    if (resp.has_response) {
      std::cout << resp.body << "\n";
      std::cout.flush();
    }
  }
  return 0;
}

}  // namespace bf::mcp
