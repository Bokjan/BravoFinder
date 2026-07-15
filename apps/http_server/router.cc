// router.cc — endpoint table, ?cycle= parsing, and status selection for bf-http.

#include "router.h"

#include <charconv>
#include <string_view>
#include <utility>

#include "io/cache/bfdb_inventory.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "work.h"

namespace bf::http {

namespace {

// The HTTP path each shared handler is exposed under (design section 二). Keyed
// by the stable bf::service handler name.
struct Route {
  const char* name;
  const char* path;
};
const Route kRoutes[] = {
    {"find_routes", "/v1/routes"},           {"parse_route", "/v1/parse-route"},
    {"lookup_waypoints", "/v1/waypoints"},   {"lookup_airports", "/v1/airports"},
    {"lookup_procedures", "/v1/procedures"}, {"lookup_procedure_legs", "/v1/procedure-legs"},
    {"lookup_airways", "/v1/airways"},       {"lookup_navaid_detail", "/v1/navaid-detail"},
    {"lookup_holds", "/v1/holds"},
};

// Parse an optional ?cycle=NNNN out of the raw query string. Returns false if a
// cycle parameter is present but not a valid non-negative integer; true (with
// *out possibly still nullopt) when absent or well-formed.
bool ParseCycle(const std::string& query, std::optional<uint32_t>* out) {
  *out = std::nullopt;
  size_t i = 0;
  while (i < query.size()) {
    const size_t amp = query.find('&', i);
    const std::string_view token(query.data() + i,
                                 (amp == std::string::npos ? query.size() : amp) - i);
    const size_t eq = token.find('=');
    if (eq != std::string_view::npos && token.substr(0, eq) == "cycle") {
      const std::string_view value = token.substr(eq + 1);
      uint32_t parsed = 0;
      const char* begin = value.data();
      const char* end = value.data() + value.size();
      const auto [ptr, ec] = std::from_chars(begin, end, parsed);
      if (ec != std::errc() || ptr != end || value.empty()) {
        return false;
      }
      *out = parsed;
      return true;
    }
    if (amp == std::string::npos) {
      break;
    }
    i = amp + 1;
  }
  return true;
}

// Readiness handler: reached only when registry.Get(latest) succeeded, so the
// server can open and serve the newest cycle. A resolve failure is turned into
// 503 by the work layer (cycle_error_status), never reaching this handler.
bf::service::HandlerResult ReadyHandler(const rapidjson::Value& /*args*/,
                                        const bf::NavDatabase& /*db*/) {
  return {R"({"status":"ready"})", 200};
}

}  // namespace

Router::Router(bf::service::NavDatabaseRegistry& registry, uv_loop_t* loop)
    : registry_(registry), loop_(loop) {
  // Index the shared handlers by name, then bind each to its HTTP path.
  std::unordered_map<std::string, bf::service::QueryHandler> by_name;
  for (bf::service::NamedHandler& nh : bf::service::MakeHandlers()) {
    by_name.emplace(std::move(nh.name), std::move(nh.handler));
  }
  for (const Route& r : kRoutes) {
    auto it = by_name.find(r.name);
    if (it != by_name.end()) {
      routes_.emplace(r.path, std::move(it->second));
    }
  }
}

void Router::Handle(std::shared_ptr<Connection> conn, const HttpRequest& req) {
  // Liveness: the process is up, so always 200 -- no database touched.
  if (req.method == "GET" && req.path == "/healthz") {
    conn->WriteResponse(200, R"({"status":"ok"})", req.keep_alive);
    return;
  }
  // The cycle list is an in-memory read of the (immutable) inventory.
  if (req.method == "GET" && req.path == "/v1/cycles") {
    conn->WriteResponse(200, SerializeCycles(), req.keep_alive);
    return;
  }
  // Readiness: opening the latest cycle may do disk I/O, so offload it; 503 if
  // it cannot be resolved.
  if (req.method == "GET" && req.path == "/readyz") {
    rapidjson::Document empty;
    empty.SetObject();
    QueueQuery(std::move(conn), loop_, registry_, ReadyHandler, std::move(empty), std::nullopt,
               req.keep_alive, 503);
    return;
  }

  // Query endpoints: POST + JSON body. An unmatched (method, path) is 404.
  auto it = routes_.find(req.path);
  if (req.method != "POST" || it == routes_.end()) {
    conn->WriteResponse(404, bf::service::JsonError("not found"), req.keep_alive);
    return;
  }
  // ?cycle=NNNN selects the database; malformed cycle is a 400 before any work.
  std::optional<uint32_t> cycle;
  if (!ParseCycle(req.query, &cycle)) {
    conn->WriteResponse(400, bf::service::JsonError("cycle must be a non-negative integer"),
                        req.keep_alive);
    return;
  }
  // The body is the handler's arguments object. Empty body => empty object;
  // malformed JSON (or a non-object) is a 400 before any work.
  rapidjson::Document args;
  if (req.body.empty()) {
    args.SetObject();
  } else {
    args.Parse(req.body.c_str());
    if (args.HasParseError() || !args.IsObject()) {
      conn->WriteResponse(400, bf::service::JsonError("request body must be a JSON object"),
                          req.keep_alive);
      return;
    }
  }
  // Offload: an unknown/unserved cycle is the client's error (400).
  QueueQuery(std::move(conn), loop_, registry_, it->second, std::move(args), cycle, req.keep_alive,
             400);
}

std::string Router::SerializeCycles() const {
  const BfdbInventory& inv = registry_.inventory();
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("cycles");
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
  writer.EndObject();
  return buffer.GetString();
}

}  // namespace bf::http
