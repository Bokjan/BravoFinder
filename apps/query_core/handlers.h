// handlers.h — the transport-neutral query handlers shared by the MCP and HTTP
// apps.
//
// Each handler takes the parsed request "arguments" object and a read-only
// NavDatabase and returns a HandlerResult: a JSON body plus an HTTP-style status
// code. Handlers are pure with respect to server state (no globals): they only
// read the database, so they are safe to call concurrently on one instance
// (NavDatabase contract B). Both transports reuse them -- MCP maps
// is_error = (status >= 400); HTTP uses the status directly.

#pragma once

#include <functional>
#include <string>
#include <vector>

#include "io/nav_database.h"
#include "rapidjson/document.h"

namespace bf::service {

// A JSON body plus an HTTP-style status. status is 200 on success; 400 for a
// bad request (missing/invalid arguments), 404 when nothing matched, 422 when a
// syntactically valid request could not be satisfied (no route, bad route
// token). MCP only needs is_error = (status >= 400); HTTP uses status directly.
struct HandlerResult {
  std::string body;
  int status;
};

using QueryHandler =
    std::function<HandlerResult(const rapidjson::Value& args, const bf::NavDatabase& db)>;

// A handler bound to its stable name. The name is the contract both transports
// key off: MCP attaches its per-tool description/schema by name, HTTP routes an
// endpoint to a handler by name.
struct NamedHandler {
  std::string name;
  QueryHandler handler;
};

// Build every query handler, in a stable display order (find_routes first, then
// parse_route, then the batch lookups). Returned by value so callers own the
// vector -- no function-level static mutable state.
std::vector<NamedHandler> MakeHandlers();

// Build an error JSON payload `{"error":"<message>"}` with RapidJSON's Writer so
// the message is auto-escaped. Handler error messages may carry user-controlled
// strings (an unknown departure airport, a bad route token, an unknown id), so
// this is the only sanctioned way to emit such a payload.
std::string JsonError(const std::string& message);

}  // namespace bf::service
