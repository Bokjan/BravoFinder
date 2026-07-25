// SPDX-License-Identifier: LGPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "core/routing/route_parser.h"
#include "core/routing/route_string.h"
#include "core/util/string_util.h"
#include "io/graph_builder.h"
#include "io/nav_database.h"

namespace bf {

namespace {

// A resolved waypoint token during route parsing: its vertex and coordinate.
struct ResolvedFix {
  int vertex = -1;
  Coordinate coord;
};

// Derive the phase-split summary fields from the already-built legs, matching
// FindRoutes semantics. The airport<->network procedure legs carry the literal
// "SID"/"STAR" keyword in `via` (the procedure name, if any, lives in
// route.sid/star and cannot be recovered from a bare keyword), so the split is
// read off `via`, not off route.sid/star being non-empty. Everything else is
// enroute. Connection is kProcedure when such a leg exists, else kDirect;
// ParseRoute never produces the radar-vector fallback.
void FinalizePhaseSplit(Route& route, bool dep_via_sid, bool arr_via_star) {
  double dep = 0.0;
  double arr = 0.0;
  for (const RouteLeg& leg : route.legs) {
    if (leg.via == "SID") {
      dep += leg.distance_nm;
    } else if (leg.via == "STAR") {
      arr += leg.distance_nm;
    }
  }
  route.dep_distance_nm = dep;
  route.arr_distance_nm = arr;
  route.enroute_distance_nm = std::max(0.0, route.total_distance_nm - dep - arr);
  route.dep_connection = dep_via_sid ? ConnectionKind::kProcedure : ConnectionKind::kDirect;
  route.arr_connection = arr_via_star ? ConnectionKind::kProcedure : ConnectionKind::kDirect;
}

}  // namespace

Result<Route> NavDatabase::ParseRoute(const std::string& route_str) const {
  if (!builder_) {
    return Result<Route>::Err(Error(ErrorCode::kDataMissing, "database not loaded"));
  }
  const std::vector<std::string> tokens = TokenizeRoute(route_str);
  if (tokens.empty()) {
    return Result<Route>::Err(Error(ErrorCode::kNoRoute, "empty route string"));
  }
  const NavGraph& graph = builder_->graph();

  // Resolve a bare waypoint ident (or IDENT/REGION) to the match nearest a
  // reference coordinate; connectivity along the route disambiguates naturally
  // because we always resolve against the previous point. Returns vertex -1 if
  // no non-airport match exists.
  auto resolve_fix = [&](const std::string& token, const Coordinate& ref) -> ResolvedFix {
    const size_t slash = token.find('/');
    if (slash != std::string::npos) {
      const int v = builder_->VertexByIdent(Ident(token.substr(0, slash), token.substr(slash + 1)));
      if (v < 0 || builder_->IsAirport(v)) {
        return {};
      }
      return {v, graph.CoordOf(v)};
    }
    int best = -1;
    double best_d = 0.0;
    for (const int v : builder_->VerticesByIdent(token)) {
      if (builder_->IsAirport(v)) {
        continue;
      }
      const double d = ref.DistanceTo(graph.CoordOf(v));
      if (best < 0 || d < best_d) {
        best = v;
        best_d = d;
      }
    }
    if (best < 0) {
      return {};
    }
    return {best, graph.CoordOf(best)};
  };

  // Walk airway `name` from `from` to `to`, following only edges whose
  // designators include `name` (Dijkstra restricted to that airway). Returns the
  // intermediate + destination vertices (excluding `from`) in order, or empty if
  // the airway does not connect them. Small, bounded search per airway.
  auto expand_airway = [&](const std::string& name, int from, int to) -> std::vector<int> {
    const int n = graph.VertexCount();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    using QN = std::pair<double, int>;
    std::priority_queue<QN, std::vector<QN>, std::greater<>> pq;
    dist[from] = 0.0;
    pq.push({0.0, from});
    while (!pq.empty()) {
      const auto [d, u] = pq.top();
      pq.pop();
      if (d > dist[u]) {
        continue;
      }
      if (u == to) {
        break;
      }
      for (const GraphEdge* e = graph.EdgesBegin(u); e != graph.EdgesEnd(u); ++e) {
        if (e->airway_id == 0) {
          continue;  // DCT edge is not on any named airway
        }
        bool on_airway = false;
        for (const std::string& d2 : SplitDesignators(builder_->AirwayName(e->airway_id))) {
          if (d2 == name) {
            on_airway = true;
            break;
          }
        }
        if (!on_airway) {
          continue;
        }
        const double nd = d + e->distance_nm;
        if (nd < dist[e->to]) {
          dist[e->to] = nd;
          prev[e->to] = u;
          pq.push({nd, e->to});
        }
      }
    }
    if (std::isinf(dist[to])) {
      return {};  // airway does not connect from -> to
    }
    std::vector<int> chain;
    for (int at = to; at != from && at != -1; at = prev[at]) {
      chain.push_back(at);
    }
    std::reverse(chain.begin(), chain.end());
    return chain;
  };

  Route route;
  std::vector<int> point_vertices;  // graph vertices for the enroute points

  // --- Optional leading departure airport. ---
  size_t i = 0;
  std::string dep_airport;
  if (builder_->VertexByAirport(tokens.front()) >= 0) {
    dep_airport = tokens.front();
    i = 1;
  }

  // --- Optional trailing arrival airport. ---
  std::string arr_airport;
  size_t end = tokens.size();
  if (end > i + 1 && builder_->VertexByAirport(tokens.back()) >= 0) {
    arr_airport = tokens.back();
    end = tokens.size() - 1;
  }

  // Reference coordinate for disambiguation: the departure airport if present,
  // else world origin (the first fix then resolves to its globally nearest
  // match, refined by connectivity on subsequent fixes).
  Coordinate ref =
      dep_airport.empty() ? Coordinate{} : graph.CoordOf(builder_->VertexByAirport(dep_airport));

  // --- Optional leading SID and trailing STAR. ---
  // Helper: does `icao` publish a procedure of `type` named `proc_name`? Used to
  // accept a hand-filed procedure name adjacent to its airport (the literal
  // "SID"/"STAR" keyword is handled separately below).
  auto airport_has_procedure = [&](const std::string& icao, const std::string& proc_name,
                                   ProcedureType type) -> bool {
    if (icao.empty()) {
      return false;
    }
    const CifpData* cifp = ProceduresFor(icao);
    if (cifp == nullptr) {
      return false;
    }
    for (const Procedure& p : cifp->procedures) {
      if (p.type == type && p.name == proc_name) {
        return true;
      }
    }
    return false;
  };

  // A procedure connector is recognized adjacent to its airport in either form:
  // the literal keyword "SID"/"STAR" that FindRoutes emits (the name is not
  // recoverable from the string, so it stays empty), or an actual published
  // procedure name (accepted for hand-filed plans and preserved in route.sid/
  // star). Anything else is treated as a fix. The rebuilt leg's `via` always
  // carries the literal keyword, matching FindRoutes output.
  std::string sid_name;
  bool dep_via_sid = false;
  if (i < end && !dep_airport.empty() &&
      (tokens[i] == "SID" || airport_has_procedure(dep_airport, tokens[i], ProcedureType::kSid))) {
    if (tokens[i] != "SID") {
      sid_name = tokens[i];
    }
    dep_via_sid = true;
    ++i;
  }
  std::string star_name;
  bool arr_via_star = false;
  if (end > i && !arr_airport.empty() &&
      (tokens[end - 1] == "STAR" ||
       airport_has_procedure(arr_airport, tokens[end - 1], ProcedureType::kStar))) {
    if (tokens[end - 1] != "STAR") {
      star_name = tokens[end - 1];
    }
    arr_via_star = true;
    --end;
  }

  // Pure direct airport-to-airport link: "DEP DCT ARR" with no enroute fix -- the
  // only no-fix shape we accept. The middle is exactly the "DCT" connector between
  // the two airports; emit a single direct leg. Anything that does not both start
  // and end at an airport still requires an enroute fix and falls through to the
  // loop below (and fails there if it has none).
  //
  // The SID/STAR recognition above may have consumed a leading "SID" / trailing
  // "STAR" before reaching here. A no-fix shape that also names a procedure (e.g.
  // "DEP SID DCT ARR") is not a valid filed route -- a SID/STAR leg always pairs
  // the airport with a transition fix, never with the far airport directly -- so
  // it must NOT take this shortcut: the shortcut emits a bare "DCT" leg and leaves
  // route.sid/star empty, silently dropping the procedure that was just parsed.
  // Guard against that by requiring no procedure connector was recognized; such
  // shapes then fall through and fail cleanly in the enroute loop.
  if (!dep_airport.empty() && !arr_airport.empty() && end - i == 1 && tokens[i] == "DCT" &&
      !dep_via_sid && !arr_via_star) {
    const int dep_v = builder_->VertexByAirport(dep_airport);
    const int arr_v = builder_->VertexByAirport(arr_airport);
    const double d = graph.CoordOf(dep_v).DistanceTo(graph.CoordOf(arr_v));
    route.points.push_back(RoutePoint{dep_airport, graph.CoordOf(dep_v)});
    route.points.push_back(RoutePoint{arr_airport, graph.CoordOf(arr_v)});
    route.legs.push_back(RouteLeg{dep_airport, arr_airport, "DCT", d, {}});
    route.total_distance_nm += d;
    route.route_string = BuildRouteString(route.points.front().ident, route.legs);
    FinalizePhaseSplit(route, dep_via_sid, arr_via_star);
    return Result<Route>::Ok(std::move(route));
  }

  // --- Middle: FIX (AWY FIX | DCT FIX)* --------------------------------------
  // Track the previous fix vertex/coord; connectors (airway names, "DCT") apply
  // to the hop from the previous fix to the next.
  int prev_vertex = -1;
  bool expect_fix = true;
  std::string pending_connector;  // "" until a connector is seen; "DCT" or airway

  auto add_point = [&](int vertex) {
    point_vertices.push_back(vertex);
    route.points.push_back(RoutePoint{builder_->IdentOf(vertex).ident, graph.CoordOf(vertex)});
    prev_vertex = vertex;
    ref = graph.CoordOf(vertex);
  };

  for (; i < end; ++i) {
    const std::string& tok = tokens[i];
    const bool is_airway = airway_index_.find(tok) != airway_index_.end();

    if (expect_fix) {
      // Expecting a fix. A leading connector before any fix is an error.
      const ResolvedFix rf = resolve_fix(tok, ref);
      if (rf.vertex < 0) {
        return Result<Route>::Err(Error(
            ErrorCode::kNoRoute, "token '" + tok + "' is not a known waypoint at this position"));
      }
      if (prev_vertex < 0) {
        // First fix: just record it.
        add_point(rf.vertex);
      } else if (pending_connector == "DCT" || pending_connector.empty()) {
        // Direct leg from the previous fix.
        const double d = graph.CoordOf(prev_vertex).DistanceTo(rf.coord);
        route.legs.push_back(RouteLeg{builder_->IdentOf(prev_vertex).ident,
                                      builder_->IdentOf(rf.vertex).ident,
                                      "DCT",
                                      d,
                                      {}});
        route.total_distance_nm += d;
        add_point(rf.vertex);
      } else {
        // Airway leg: expand the airway from prev to this fix.
        const std::vector<int> chain = expand_airway(pending_connector, prev_vertex, rf.vertex);
        if (chain.empty()) {
          return Result<Route>::Err(
              Error(ErrorCode::kNoRoute, "airway '" + pending_connector + "' does not connect " +
                                             builder_->IdentOf(prev_vertex).ident + " to " + tok));
        }
        int hop_from = prev_vertex;
        for (const int v : chain) {
          const double d = graph.CoordOf(hop_from).DistanceTo(graph.CoordOf(v));
          route.legs.push_back(RouteLeg{builder_->IdentOf(hop_from).ident,
                                        builder_->IdentOf(v).ident,
                                        pending_connector,
                                        d,
                                        {}});
          route.total_distance_nm += d;
          add_point(v);
          hop_from = v;
        }
      }
      pending_connector.clear();
      expect_fix = false;
    } else {
      // Expecting a connector: an airway name or DCT.
      if (tok == "DCT") {
        pending_connector = "DCT";
      } else if (is_airway) {
        pending_connector = tok;
      } else {
        // Two fixes in a row with no connector: treat as an implicit DCT so
        // "FIX FIX" is accepted (common in filed plans), then re-handle this
        // token as a fix.
        pending_connector = "DCT";
        --i;  // reprocess tok as a fix on the next iteration
      }
      expect_fix = true;
    }
  }

  if (point_vertices.empty()) {
    return Result<Route>::Err(Error(ErrorCode::kNoRoute, "route has no waypoints"));
  }
  // A trailing connector with no following fix (e.g. "MCI J24" or "... PSB J60").
  // The loop's invariant is expect_fix==false <=> pending_connector.empty(): a
  // dangling connector always leaves expect_fix==true, so checking !expect_fix
  // here would be dead. Gate on the pending connector alone.
  if (!pending_connector.empty()) {
    return Result<Route>::Err(
        Error(ErrorCode::kNoRoute, "route ends with '" + pending_connector + "' but no fix"));
  }

  // --- Prepend the departure airport / SID and append the arrival / STAR. ---
  if (!dep_airport.empty()) {
    const int apt = builder_->VertexByAirport(dep_airport);
    const double d = graph.CoordOf(apt).DistanceTo(graph.CoordOf(point_vertices.front()));
    route.points.insert(route.points.begin(), RoutePoint{dep_airport, graph.CoordOf(apt)});
    route.legs.insert(route.legs.begin(), RouteLeg{dep_airport,
                                                   builder_->IdentOf(point_vertices.front()).ident,
                                                   dep_via_sid ? "SID" : "DCT",
                                                   d,
                                                   {}});
    route.total_distance_nm += d;
    route.sid = sid_name;
  }
  if (!arr_airport.empty()) {
    const int apt = builder_->VertexByAirport(arr_airport);
    const double d = graph.CoordOf(point_vertices.back()).DistanceTo(graph.CoordOf(apt));
    route.points.push_back(RoutePoint{arr_airport, graph.CoordOf(apt)});
    route.legs.push_back(RouteLeg{builder_->IdentOf(point_vertices.back()).ident,
                                  arr_airport,
                                  arr_via_star ? "STAR" : "DCT",
                                  d,
                                  {}});
    route.total_distance_nm += d;
    route.star = star_name;
  }

  // Rebuild the canonical filed route string from the resolved legs (folds
  // consecutive same-airway legs, matching FindRoutes output).
  const std::string first_point = route.points.empty() ? "" : route.points.front().ident;
  route.route_string = BuildRouteString(first_point, route.legs);
  FinalizePhaseSplit(route, dep_via_sid, arr_via_star);
  return Result<Route>::Ok(std::move(route));
}

}  // namespace bf
