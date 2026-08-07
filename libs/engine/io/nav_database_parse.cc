// SPDX-License-Identifier: LGPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "core/base/string_util.h"
#include "core/domain/nav_tokens.h"
#include "core/routing/route_parser.h"
#include "core/routing/route_string.h"
#include "io/build/graph_builder.h"
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
    if (leg.via == kSidToken) {
      dep += leg.distance_nm;
    } else if (leg.via == kStarToken) {
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
    return Result<Route>::Err(Error(ErrorCode::kRouteParseError, "empty route string"));
  }
  // A filed route is bracketed by airports: departure ICAO first, arrival last.
  if (builder_->VertexByAirport(tokens.front()) < 0) {
    return Result<Route>::Err(
        Error(ErrorCode::kRouteParseError,
              "route must start with a departure airport ICAO (first token '" + tokens.front() +
                  "' is not a known airport)"));
  }
  if (tokens.size() < 2) {
    return Result<Route>::Err(
        Error(ErrorCode::kRouteParseError,
              "route has only a departure airport; expected an arrival airport and a connector"));
  }
  if (builder_->VertexByAirport(tokens.back()) < 0) {
    return Result<Route>::Err(Error(ErrorCode::kRouteParseError,
                                    "route must end with an arrival airport ICAO (last token '" +
                                        tokens.back() + "' is not a known airport)"));
  }
  if (tokens.size() == 2) {
    // "DEP ARR": two airports with nothing between them. Per the filed-plan
    // contract every adjacent pair needs an explicit connector, so the only
    // no-fix shape is "DEP DCT ARR". Report the missing connector rather than
    // letting the enroute loop mistake the arrival for a bare waypoint.
    return Result<Route>::Err(
        Error(ErrorCode::kRouteParseError, "airport pair '" + tokens.front() + " " + tokens.back() +
                                               "' has no connector (expected '" + tokens.front() +
                                               " DCT " + tokens.back() + "')"));
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

  // Dijkstra scratch for expand_airway, reused across every airway leg in this
  // route. The vectors are allocated once (V ~ 270k, ~3 MB) instead of per leg;
  // a generation stamp avoids re-clearing dist each leg -- a vertex's dist is
  // valid only for the current generation, so untouched vertices read as +inf
  // without being wiped. Mirrors the A* SearchWorkspace reuse pattern.
  const int n = graph.VertexCount();
  std::vector<double> dist(n);
  std::vector<int> prev(n, -1);
  std::vector<int> gen(n, 0);
  int cur_gen = 0;

  // Walk airway `name` from `from` to `to`, following only edges whose
  // designators include `name` (Dijkstra restricted to that airway). Returns the
  // intermediate + destination vertices (excluding `from`) in order, or empty if
  // the airway does not connect them. Small, bounded search per airway.
  auto expand_airway = [&](const std::string& name, int from, int to) -> std::vector<int> {
    ++cur_gen;  // new generation: every dist[] reads as +inf until touched
    auto dist_of = [&](int v) -> double {
      return gen[v] == cur_gen ? dist[v] : std::numeric_limits<double>::infinity();
    };
    using QN = std::pair<double, int>;
    std::priority_queue<QN, std::vector<QN>, std::greater<>> pq;
    gen[from] = cur_gen;
    dist[from] = 0.0;
    prev[from] = -1;
    pq.push({0.0, from});
    while (!pq.empty()) {
      const auto [d, u] = pq.top();
      pq.pop();
      if (d > dist_of(u)) {
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
        if (nd < dist_of(e->to)) {
          gen[e->to] = cur_gen;
          dist[e->to] = nd;
          prev[e->to] = u;
          pq.push({nd, e->to});
        }
      }
    }
    if (std::isinf(dist_of(to))) {
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
  bool arr_explicit = false;  // a STAR or explicit "DCT" precedes the airport
  size_t end = tokens.size();
  if (end > i + 1 && builder_->VertexByAirport(tokens.back()) >= 0) {
    arr_airport = tokens.back();
    end = tokens.size() - 1;
    // The airport-append step below emits the final leg, so a trailing "DCT
    // AIRPORT" is redundant -- consume it. Guard end > i + 1 leaves the "DEP
    // DCT ARR" pure-direct shortcut its DCT.
    if (end > i + 1 && tokens[end - 1] == kDctToken) {
      --end;
      arr_explicit = true;
    }
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
                                   ProcedureType type) -> Result<bool> {
    if (icao.empty()) {
      return Result<bool>::Ok(false);
    }
    Result<const CifpData*> cifp_r = ProceduresFor(icao);
    if (!cifp_r) {
      return Result<bool>::Err(std::move(cifp_r).error());
    }
    const CifpData* cifp = cifp_r.value();
    if (cifp == nullptr) {
      return Result<bool>::Ok(false);
    }
    for (const Procedure& p : cifp->procedures) {
      if (p.type == type && p.name == proc_name) {
        return Result<bool>::Ok(true);
      }
    }
    return Result<bool>::Ok(false);
  };

  // A procedure connector is recognized adjacent to its airport in either form:
  // the literal keyword "SID"/"STAR" that FindRoutes emits (the name is not
  // recoverable from the string, so it stays empty), or an actual published
  // procedure name (accepted for hand-filed plans and preserved in route.sid/
  // star). Anything else is treated as a fix. The rebuilt leg's `via` always
  // carries the literal keyword, matching FindRoutes output.
  std::string sid_name;
  bool dep_via_sid = false;
  if (i < end && !dep_airport.empty()) {
    if (tokens[i] == kSidToken) {
      dep_via_sid = true;
      ++i;
    } else {
      Result<bool> has_sid = airport_has_procedure(dep_airport, tokens[i], ProcedureType::kSid);
      if (!has_sid) {
        return Result<Route>::Err(std::move(has_sid).error());
      }
      if (has_sid.value()) {
        sid_name = tokens[i];
        dep_via_sid = true;
        ++i;
      }
    }
  }
  // The airport-prepend step below emits the first leg, so a leading "AIRPORT
  // DCT" is redundant -- consume it. dep_explicit tracks that a SID or DCT
  // connects the airport to the first fix. Guard i + 1 < end leaves the "DEP
  // DCT ARR" pure-direct shortcut its DCT.
  bool dep_explicit = dep_via_sid;
  if (!dep_airport.empty() && !dep_via_sid && i < end && tokens[i] == kDctToken && i + 1 < end) {
    ++i;
    dep_explicit = true;
  }
  std::string star_name;
  bool arr_via_star = false;
  if (end > i && !arr_airport.empty()) {
    if (tokens[end - 1] == kStarToken) {
      arr_via_star = true;
      --end;
      arr_explicit = true;
    } else {
      Result<bool> has_star =
          airport_has_procedure(arr_airport, tokens[end - 1], ProcedureType::kStar);
      if (!has_star) {
        return Result<Route>::Err(std::move(has_star).error());
      }
      if (has_star.value()) {
        star_name = tokens[end - 1];
        arr_via_star = true;
        --end;
        arr_explicit = true;
      }
    }
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
  if (!dep_airport.empty() && !arr_airport.empty() && end - i == 1 && tokens[i] == kDctToken &&
      !dep_via_sid && !arr_via_star) {
    const int dep_v = builder_->VertexByAirport(dep_airport);
    const int arr_v = builder_->VertexByAirport(arr_airport);
    const double d = graph.CoordOf(dep_v).DistanceTo(graph.CoordOf(arr_v));
    route.points.push_back(RoutePoint{dep_airport, graph.CoordOf(dep_v)});
    route.points.push_back(RoutePoint{arr_airport, graph.CoordOf(arr_v)});
    route.legs.push_back(RouteLeg{dep_airport, arr_airport, std::string(kDctToken), d, {}});
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
    // Airway designators are stored uppercase, so normalize the token before
    // both the is_airway probe and the connector handed to expand_airway
    // (which compares against the stored uppercase designator). Fix lookups
    // keep the original token so error messages show what the user typed.
    const std::string up_tok = ToUpper(tok);
    const bool is_airway = FindAirway(up_tok) != nullptr;

    if (expect_fix) {
      // Expecting a fix. A leading connector before any fix is an error.
      const ResolvedFix rf = resolve_fix(tok, ref);
      if (rf.vertex < 0) {
        return Result<Route>::Err(
            Error(ErrorCode::kRouteParseError,
                  "token '" + tok + "' is not a known waypoint at this position"));
      }
      if (prev_vertex < 0) {
        // First fix: just record it.
        add_point(rf.vertex);
      } else if (pending_connector == kDctToken || pending_connector.empty()) {
        // Direct leg from the previous fix.
        const double d = graph.CoordOf(prev_vertex).DistanceTo(rf.coord);
        route.legs.push_back(RouteLeg{builder_->IdentOf(prev_vertex).ident,
                                      builder_->IdentOf(rf.vertex).ident,
                                      std::string(kDctToken),
                                      d,
                                      {}});
        route.total_distance_nm += d;
        add_point(rf.vertex);
      } else {
        // Airway leg: expand the airway from prev to this fix.
        const std::vector<int> chain = expand_airway(pending_connector, prev_vertex, rf.vertex);
        if (chain.empty()) {
          return Result<Route>::Err(Error(ErrorCode::kRouteParseError,
                                          "airway '" + pending_connector + "' does not connect " +
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
      if (tok == kDctToken) {
        pending_connector = std::string(kDctToken);
      } else if (is_airway) {
        pending_connector = up_tok;
      } else {
        // No connector between two fixes: ICAO requires one between every pair
        // of waypoints. Reject rather than synthesize an implicit DCT.
        return Result<Route>::Err(Error(
            ErrorCode::kRouteParseError,
            "two consecutive fixes '" + tok + "' with no connector (expected DCT or airway)"));
      }
      expect_fix = true;
    }
  }

  if (point_vertices.empty()) {
    return Result<Route>::Err(Error(ErrorCode::kRouteParseError, "route has no waypoints"));
  }
  // A trailing connector with no following fix (e.g. "MCI J24" or "... PSB J60").
  // The loop's invariant is expect_fix==false <=> pending_connector.empty(): a
  // dangling connector always leaves expect_fix==true, so checking !expect_fix
  // here would be dead. Gate on the pending connector alone.
  if (!pending_connector.empty()) {
    return Result<Route>::Err(Error(ErrorCode::kRouteParseError,
                                    "route ends with '" + pending_connector + "' but no fix"));
  }

  // Every airport<->fix boundary needs an explicit connector. The "DEP DCT ARR"
  // shortcut returns before here, so any route with a fix requires both ends
  // explicit; the append step must not synthesize a DCT leg for a bare "AIRPORT
  // FIX" or "FIX AIRPORT".
  if (!dep_airport.empty() && !dep_explicit) {
    return Result<Route>::Err(
        Error(ErrorCode::kRouteParseError,
              "departure airport '" + dep_airport +
                  "' has no connector to the first fix (expected DCT or SID)"));
  }
  if (!arr_airport.empty() && !arr_explicit) {
    return Result<Route>::Err(
        Error(ErrorCode::kRouteParseError,
              "arrival airport '" + arr_airport +
                  "' has no connector to the last fix (expected DCT or STAR)"));
  }

  // --- Prepend the departure airport / SID and append the arrival / STAR. ---
  if (!dep_airport.empty()) {
    const int apt = builder_->VertexByAirport(dep_airport);
    const double d = graph.CoordOf(apt).DistanceTo(graph.CoordOf(point_vertices.front()));
    route.points.insert(route.points.begin(), RoutePoint{dep_airport, graph.CoordOf(apt)});
    route.legs.insert(route.legs.begin(),
                      RouteLeg{dep_airport,
                               builder_->IdentOf(point_vertices.front()).ident,
                               dep_via_sid ? std::string(kSidToken) : std::string(kDctToken),
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
                                  arr_via_star ? std::string(kStarToken) : std::string(kDctToken),
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
