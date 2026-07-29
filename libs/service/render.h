// SPDX-License-Identifier: MIT
// render.h — result rendering for the query layer, shared by the JSON-args
// handlers (MCP / HTTP) and the CLI.
//
// Every query entry point (queries.h) runs the engine call, renders the result
// in the requested OutputFormat, and wraps it as a HandlerResult. JSON is the
// wire format the transports ship (a bare array / object body; elapsed_ms
// travels out-of-band in HandlerResult). Text is the human-readable form the
// CLI prints by default, ported verbatim from the former CLI printers so script
// output is byte-for-byte unchanged.

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/query/query_types.h"
#include "core/routing/route.h"

namespace bf::service {

// The format a query entry point should render. Transports always pass kJson;
// the CLI passes the user's --format choice.
enum class OutputFormat {
  kJson,
  kText,
};

// ---- Routes (find_routes / parse_route) -------------------------------------
// JSON: a bare array of route objects (the shape the transports ship; elapsed_ms
// is out-of-band). Text: one block per route (with a multi-route header) and a
// final "Query elapsed" line.
std::string RenderRoutes(OutputFormat fmt, const std::vector<bf::Route>& routes,
                         uint32_t elapsed_ms);

// A single route (parse_route). JSON: the route object directly -- parse_route
// always yields exactly one route, so the transports ship it as a single object
// (matching the /v1/parse-route and MCP parse_route contracts), not a one-element
// array. Text: the single route block plus a final "Query elapsed" line, identical
// to RenderRoutes with one route.
std::string RenderRoute(OutputFormat fmt, const bf::Route& route, uint32_t elapsed_ms);

// ---- Batch lookups ----------------------------------------------------------
// JSON: an array parallel to `ids` -- null (optional lookups) or an empty array
// (grouped lookups) for a not-found id. Text: one line per match, "<id>: not
// found" for a miss. `ids` is parallel to `results` and labels the text output.
std::string RenderWaypoints(OutputFormat fmt, const std::vector<std::string>& ids,
                            const std::vector<std::vector<bf::WaypointInfo>>& results);
std::string RenderAirports(OutputFormat fmt, const std::vector<std::string>& ids,
                           const std::vector<std::optional<bf::AirportInfo>>& results);
std::string RenderProcedures(OutputFormat fmt, const std::vector<std::string>& ids,
                             const std::vector<std::optional<bf::AirportProcedures>>& results);
std::string RenderAirways(OutputFormat fmt, const std::vector<std::string>& ids,
                          const std::vector<std::optional<bf::AirwayInfo>>& results);
std::string RenderNavaidDetails(OutputFormat fmt, const std::vector<std::string>& ids,
                                const std::vector<std::vector<bf::NavaidDetailInfo>>& results);
std::string RenderHolds(OutputFormat fmt, const std::vector<std::string>& ids,
                        const std::vector<std::vector<bf::HoldInfo>>& results);

// ---- Single procedure detail (lookup_procedure_legs) ------------------------
// JSON: the procedure-detail object. Text: the per-transition / per-leg block.
std::string RenderProcedureDetail(OutputFormat fmt, const bf::AirportProcedureDetail& d);

// ---- Mixed procedure selectors (CLI `query procedure`) ----------------------
// Parallel vectors: for each selector exactly one of summaries[i] / details[i]
// is set (a bare airport => summary; an "airport/procedure" => detail); both
// nullopt => a miss, labeled by labels[i]. JSON: one array, heterogeneous
// elements (summary object / detail object / null). Text: per-selector blocks
// (summary list, per-leg detail, or "<label>: not found").
std::string RenderProceduresMixed(
    OutputFormat fmt, const std::vector<std::string>& labels,
    const std::vector<std::optional<bf::AirportProcedures>>& summaries,
    const std::vector<std::optional<bf::AirportProcedureDetail>>& details);

// ---- Error payload ----------------------------------------------------------
// JSON: {"error":"<message>"}. Text: "error: <message>\n".
std::string RenderError(OutputFormat fmt, const std::string& message);

// Build an error JSON payload `{"error":"<message>"}` with RapidJSON's Writer so
// the message is auto-escaped. Handler error messages may carry user-controlled
// strings (an unknown departure airport, a bad route token, an unknown id), so
// this is the only sanctioned way to emit such a payload. Lives here (not in
// handlers.h) so render.cc no longer has to reach back into the handlers layer.
std::string JsonError(const std::string& message);

}  // namespace bf::service
