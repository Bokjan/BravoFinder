// SPDX-License-Identifier: MIT
// queries.h — the typed query entry points of the query layer.
//
// Each entry runs the engine call, renders the result in the requested
// OutputFormat, and wraps it as a HandlerResult (body + HTTP-style status +
// elapsed_ms). The JSON-args handlers (MakeHandlers, for MCP / HTTP) and the
// CLI both build on these: transports always pass kJson, the CLI passes the
// user's --format choice. Adding a query capability means adding one entry here
// and one MakeHandlers adapter; both transports and the CLI pick it up.

#pragma once

#include <string>
#include <vector>

#include "core/routing/route_request.h"
#include "handlers.h"
#include "io/nav_database.h"
#include "render.h"

namespace bf::service {

// find_routes / parse_route.
HandlerResult FindRoutes(const bf::NavDatabase& db, const bf::RouteRequest& request,
                         OutputFormat fmt);
HandlerResult ParseRoute(const bf::NavDatabase& db, const std::string& route_str, OutputFormat fmt);

// Batch lookups. Each takes an id list and returns a body parallel to it; status
// is 404 when every id missed, 200 on a partial or full hit.
HandlerResult LookupWaypoints(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                              OutputFormat fmt);
HandlerResult LookupAirports(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                             OutputFormat fmt);
HandlerResult LookupProcedures(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                               OutputFormat fmt);
HandlerResult LookupAirways(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                            OutputFormat fmt);
HandlerResult LookupNavaidDetails(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                                  OutputFormat fmt);
HandlerResult LookupHolds(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                          OutputFormat fmt);

// Per-leg detail of one named procedure. Status 404 when no such procedure.
HandlerResult LookupProcedureLegs(const bf::NavDatabase& db, const std::string& airport,
                                  const std::string& procedure, OutputFormat fmt);

// The CLI `query procedure` kind accepts a mix of selectors in one batch: a bare
// airport lists its procedure summaries, an "airport/procedure" pair prints that
// named procedure's per-leg detail. Rendered as one array (json, heterogeneous
// elements) or per-id text blocks. Status 404 when every selector missed.
struct ProcedureSelector {
  std::string airport;
  std::string procedure;  // empty => list the airport's procedure summaries
};
HandlerResult LookupProceduresMixed(const bf::NavDatabase& db,
                                    const std::vector<ProcedureSelector>& selectors,
                                    OutputFormat fmt);

}  // namespace bf::service
