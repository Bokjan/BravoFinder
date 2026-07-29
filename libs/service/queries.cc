// SPDX-License-Identifier: MIT
// queries.cc — typed query entry points (see queries.h).
//
// Each entry times the engine call, renders via render.h, and maps the outcome
// to a HandlerResult. The status rules (404 for a wholly-missing lookup, 422
// for a well-formed but unsatisfiable route/parse) match the former handlers.cc
// so MCP / HTTP behavior is unchanged; the CLI reuses the same path for its
// --format json and --format text output.

#include "queries.h"

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "core/routing/route.h"
#include "io/nav_database.h"
#include "render.h"

namespace bf::service {

namespace {

// HTTP-style status codes (see HandlerResult in handlers.h for the semantics).
constexpr int kOk = 200;
constexpr int kNotFound = 404;
constexpr int kUnprocessable = 422;

uint32_t ElapsedMs(std::chrono::steady_clock::time_point start) {
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   std::chrono::steady_clock::now() - start)
                                   .count());
}

}  // namespace

HandlerResult FindRoutes(const bf::NavDatabase& db, const bf::RouteRequest& request,
                         OutputFormat fmt) {
  const auto start = std::chrono::steady_clock::now();
  bf::Result<std::vector<bf::Route>> result = db.FindRoutes(request);
  const uint32_t elapsed = ElapsedMs(start);
  if (!result) {
    // A failed route computation is a semantic failure (422): the request was
    // well-formed but no route satisfies it, or an endpoint is unknown.
    return {RenderError(fmt, result.error().message), kUnprocessable, 0};
  }
  return {RenderRoutes(fmt, result.value(), elapsed), kOk, elapsed};
}

HandlerResult ParseRoute(const bf::NavDatabase& db, const std::string& route_str,
                         OutputFormat fmt) {
  const auto start = std::chrono::steady_clock::now();
  bf::Result<bf::Route> result = db.ParseRoute(route_str);
  const uint32_t elapsed = ElapsedMs(start);
  if (!result) {
    // A parse failure is a semantic failure (422): the string was given but does
    // not form a valid route. The message names the offending token.
    return {RenderError(fmt, result.error().message), kUnprocessable, 0};
  }
  return {RenderRoute(fmt, result.value(), elapsed), kOk, elapsed};
}

// ---- Batch lookups ----------------------------------------------------------

// A grouped lookup (waypoints / navaid_detail / holds) returns a group per id.
// 404 when every group is empty; 200 (with elapsed) otherwise.
template <class Info, class LookupFn, class RenderFn>
HandlerResult RunGroupedLookup(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                               OutputFormat fmt, LookupFn lookup, RenderFn render) {
  const auto start = std::chrono::steady_clock::now();
  std::vector<std::vector<Info>> results = lookup(db, ids);
  const uint32_t elapsed = ElapsedMs(start);
  const bool all_empty =
      std::all_of(results.begin(), results.end(), [](const auto& group) { return group.empty(); });
  if (all_empty) {
    return {render(fmt, ids, results), kNotFound, 0};
  }
  return {render(fmt, ids, results), kOk, elapsed};
}

// An optional lookup (airports / procedures / airways) returns an optional per
// id. 404 when every id missed; 200 (with elapsed) otherwise.
template <class Info, class LookupFn, class RenderFn>
HandlerResult RunOptionalLookup(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                                OutputFormat fmt, LookupFn lookup, RenderFn render) {
  const auto start = std::chrono::steady_clock::now();
  std::vector<std::optional<Info>> results = lookup(db, ids);
  const uint32_t elapsed = ElapsedMs(start);
  const bool all_missing =
      std::none_of(results.begin(), results.end(), [](const auto& opt) { return opt.has_value(); });
  if (all_missing) {
    return {render(fmt, ids, results), kNotFound, 0};
  }
  return {render(fmt, ids, results), kOk, elapsed};
}

HandlerResult LookupWaypoints(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                              OutputFormat fmt) {
  return RunGroupedLookup<bf::WaypointInfo>(
      db, ids, fmt,
      [](const bf::NavDatabase& d, const std::vector<std::string>& i) {
        return d.LookupWaypoints(i);
      },
      RenderWaypoints);
}

HandlerResult LookupAirports(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                             OutputFormat fmt) {
  return RunOptionalLookup<bf::AirportInfo>(
      db, ids, fmt,
      [](const bf::NavDatabase& d, const std::vector<std::string>& i) {
        return d.LookupAirports(i);
      },
      RenderAirports);
}

HandlerResult LookupProcedures(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                               OutputFormat fmt) {
  return RunOptionalLookup<bf::AirportProcedures>(
      db, ids, fmt,
      [](const bf::NavDatabase& d, const std::vector<std::string>& i) {
        return d.LookupProcedures(i);
      },
      RenderProcedures);
}

HandlerResult LookupAirways(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                            OutputFormat fmt) {
  return RunOptionalLookup<bf::AirwayInfo>(
      db, ids, fmt,
      [](const bf::NavDatabase& d, const std::vector<std::string>& i) {
        return d.LookupAirways(i);
      },
      RenderAirways);
}

HandlerResult LookupNavaidDetails(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                                  OutputFormat fmt) {
  return RunGroupedLookup<bf::NavaidDetailInfo>(
      db, ids, fmt,
      [](const bf::NavDatabase& d, const std::vector<std::string>& i) {
        return d.LookupNavaidDetails(i);
      },
      RenderNavaidDetails);
}

HandlerResult LookupHolds(const bf::NavDatabase& db, const std::vector<std::string>& ids,
                          OutputFormat fmt) {
  return RunGroupedLookup<bf::HoldInfo>(
      db, ids, fmt,
      [](const bf::NavDatabase& d, const std::vector<std::string>& i) { return d.LookupHolds(i); },
      RenderHolds);
}

HandlerResult LookupProcedureLegs(const bf::NavDatabase& db, const std::string& airport,
                                  const std::string& procedure, OutputFormat fmt) {
  const auto start = std::chrono::steady_clock::now();
  std::optional<bf::AirportProcedureDetail> detail = db.LookupProcedureDetail(airport, procedure);
  const uint32_t elapsed = ElapsedMs(start);
  if (!detail) {
    // Unknown airport, no CIFP data, or no procedure of that name: 404 rather
    // than an empty success payload.
    return {RenderError(fmt, "no procedure of that name at that airport"), kNotFound, 0};
  }
  return {RenderProcedureDetail(fmt, *detail), kOk, elapsed};
}

// ---- CLI procedure kind (mixed summary / detail selectors) ------------------

namespace {

// The label for a selector in a "not found" line: "KJFK" or "KJFK/DEEZZ5".
std::string SelectorLabel(const ProcedureSelector& s) {
  return s.procedure.empty() ? s.airport : s.airport + "/" + s.procedure;
}

}  // namespace

HandlerResult LookupProceduresMixed(const bf::NavDatabase& db,
                                    const std::vector<ProcedureSelector>& selectors,
                                    OutputFormat fmt) {
  const auto start = std::chrono::steady_clock::now();

  // Two result streams parallel to `selectors`: summaries (a bare airport) and
  // details (an "airport/procedure" pair); each selector fills exactly one, the
  // other stays nullopt. The bare-airport selectors are gathered and looked up
  // in a single batched LookupProcedures call, then scattered back by index;
  // detail selectors have no batch API and are looked up one at a time.
  std::vector<std::optional<bf::AirportProcedures>> summaries(selectors.size());
  std::vector<std::optional<bf::AirportProcedureDetail>> details(selectors.size());
  std::vector<std::string> summary_airports;
  std::vector<size_t> summary_indices;
  for (size_t i = 0; i < selectors.size(); ++i) {
    if (selectors[i].procedure.empty()) {
      summary_airports.push_back(selectors[i].airport);
      summary_indices.push_back(i);
    } else {
      details[i] = db.LookupProcedureDetail(selectors[i].airport, selectors[i].procedure);
    }
  }
  if (!summary_airports.empty()) {
    std::vector<std::optional<bf::AirportProcedures>> looked =
        db.LookupProcedures(summary_airports);
    for (size_t j = 0; j < summary_indices.size(); ++j) {
      summaries[summary_indices[j]] = std::move(looked[j]);
    }
  }
  const uint32_t elapsed = ElapsedMs(start);

  const bool any_summary = std::any_of(summaries.begin(), summaries.end(),
                                       [](const auto& opt) { return opt.has_value(); });
  const bool any_detail =
      std::any_of(details.begin(), details.end(), [](const auto& opt) { return opt.has_value(); });
  const bool all_missed = !any_summary && !any_detail;

  std::vector<std::string> labels;
  labels.reserve(selectors.size());
  for (const ProcedureSelector& s : selectors) {
    labels.push_back(SelectorLabel(s));
  }

  std::string body = RenderProceduresMixed(fmt, labels, summaries, details);
  if (all_missed) {
    return {std::move(body), kNotFound, 0};
  }
  return {std::move(body), kOk, elapsed};
}

}  // namespace bf::service
