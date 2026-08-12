// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/navdb/endpoint_planner.h"

#include <algorithm>
#include <utility>

#include "core/base/string_util.h"
#include "core/routing/route_request.h"
#include "io/build/graph_builder.h"

namespace bf {
namespace {

// Whether a procedure ref matches a requested selector. The selector is either
// a bare name ("DEEZZ5", matches any transition) or "NAME.TRANSITION"
// ("DEEZZ5.TOWIN", matches that transition exactly). Comparison is
// case-sensitive (CIFP names are already upper-case).
bool RefMatchesSelector(const ProcedureRef& ref, const std::string& selector) {
  const size_t dot = selector.find('.');
  if (dot == std::string::npos) {
    return ref.name == selector;
  }
  return ref.name == selector.substr(0, dot) && ref.transition == selector.substr(dot + 1);
}

// Filter connections in place to only those procedures matching `selector`,
// dropping any connection left with no matching procedure. Returns true if at
// least one procedure survived. A no-op returning true when the selector is
// empty (no name requested).
bool FilterConnectionsByName(std::vector<Connection>& connections, const std::string& selector) {
  if (selector.empty()) {
    return true;
  }
  bool any = false;
  for (Connection& c : connections) {
    std::vector<ProcedureRef> kept;
    for (const ProcedureRef& ref : c.procedures) {
      if (RefMatchesSelector(ref, selector)) {
        kept.push_back(ref);
      }
    }
    c.procedures = std::move(kept);
    if (!c.procedures.empty()) {
      any = true;
    }
  }
  if (any) {
    // Drop connections that no longer carry any matching procedure so the search
    // only seeds fixes reachable by the requested procedure.
    std::vector<Connection> filtered;
    for (Connection& c : connections) {
      if (!c.procedures.empty()) {
        filtered.push_back(std::move(c));
      }
    }
    connections = std::move(filtered);
  }
  return any;
}

bool IsApproachFront(const Connection& c) {
  return !c.procedures.empty() && c.procedures.front().type == ProcedureType::kApproach;
}

double EffectiveSeedNm(const Connection& c, bool soft_prefer_star) {
  double seed = c.seed_distance_nm;
  if (soft_prefer_star && IsApproachFront(c)) {
    seed += EndpointPlanner::kProcedurePreferNm;
  }
  return seed;
}

// Fold `incoming` into `by_fix` by fix_vertex. Ranking uses effective seed
// (approach + B when soft-prefer is on); stored seed_distance_nm stays raw.
void MergeConnection(std::vector<Connection>& by_fix, Connection incoming, bool soft_prefer_star) {
  for (Connection& c : by_fix) {
    if (c.fix_vertex != incoming.fix_vertex) {
      continue;
    }
    const size_t first_incoming = c.procedures.size();
    for (ProcedureRef& ref : incoming.procedures) {
      c.procedures.push_back(std::move(ref));
    }
    if (EffectiveSeedNm(incoming, soft_prefer_star) < EffectiveSeedNm(c, soft_prefer_star)) {
      c.seed_distance_nm = incoming.seed_distance_nm;
      c.bearing = incoming.bearing;
      c.approach_bearing = incoming.approach_bearing;
      c.splice_vertex = incoming.splice_vertex;
      c.splice_leg_nm = incoming.splice_leg_nm;
      if (first_incoming < c.procedures.size()) {
        std::swap(c.procedures.front(), c.procedures[first_incoming]);
      }
    }
    return;
  }
  by_fix.push_back(std::move(incoming));
}

void MergeAll(std::vector<Connection>& dst, std::vector<Connection> src, bool soft_prefer_star) {
  for (Connection& c : src) {
    MergeConnection(dst, std::move(c), soft_prefer_star);
  }
}

void SortConnectionsBySeed(std::vector<Connection>& by_fix, bool soft_prefer_star) {
  std::sort(by_fix.begin(), by_fix.end(),
            [soft_prefer_star](const Connection& a, const Connection& b) {
              const double sa = EffectiveSeedNm(a, soft_prefer_star);
              const double sb = EffectiveSeedNm(b, soft_prefer_star);
              if (sa != sb) {
                return sa < sb;
              }
              return a.fix_vertex < b.fix_vertex;
            });
}

}  // namespace

bool EndpointPlanner::SoftPreferStarActive(const EndpointPlan& plan) {
  bool any_star = false;
  bool any_apch = false;
  for (const Connection& c : plan.connections) {
    for (const ProcedureRef& ref : c.procedures) {
      if (ref.type == ProcedureType::kApproach) {
        any_apch = true;
      } else if (ref.type == ProcedureType::kStar) {
        any_star = true;
      }
    }
  }
  return any_star && any_apch;
}

std::vector<SeededEndpoint> EndpointPlanner::ToSearchEndpoints(
    const std::vector<Connection>& connections, bool soft_prefer_star) {
  std::vector<SeededEndpoint> endpoints;
  endpoints.reserve(connections.size());
  for (const Connection& c : connections) {
    endpoints.push_back(
        SeededEndpoint{c.fix_vertex, EffectiveSeedNm(c, soft_prefer_star), c.bearing});
  }
  return endpoints;
}

Result<EndpointPlan> EndpointPlanner::Plan(const GraphBuilder& builder, const RouteRequest& request,
                                           const std::string& name, bool departure,
                                           const CifpLookup& cifp_lookup) {
  const std::string up = ToUpper(name);
  EndpointPlan plan;
  const int airport = builder.VertexByAirport(up);
  if (airport >= 0) {
    plan.airport_icao = up;
    const Coordinate apt = builder.graph().CoordOf(airport);
    Result<const CifpData*> cifp_r = cifp_lookup(up);
    if (!cifp_r) {
      return Result<EndpointPlan>::Err(std::move(cifp_r).error());
    }
    const CifpData* cifp = cifp_r.value();
    if (cifp != nullptr) {
      for (const Procedure& p : cifp->procedures) {
        if (departure) {
          if (p.type == ProcedureType::kSid) {
            plan.has_procedures = true;
            break;
          }
        } else if (p.type == ProcedureType::kStar || p.type == ProcedureType::kApproach) {
          plan.has_procedures = true;
          break;
        }
      }
      const std::string& rwy = departure ? request.departure_runway : request.arrival_runway;
      const std::string& sel = departure ? request.departure_sid : request.arrival_star;
      if (departure) {
        plan.connections = ProcedureConnector::BuildDeparture(*cifp, apt, builder, rwy);
        if (!sel.empty()) {
          if (!FilterConnectionsByName(plan.connections, sel)) {
            // Named --sid: only accumulate matching off-network exit splices so
            // splice_vertex/seed bind to the requested SID, not a merge winner.
            plan.connections =
                ProcedureConnector::BuildSidSpliceDeparture(*cifp, apt, builder, rwy, sel);
            if (plan.connections.empty()) {
              plan.named_procedure_unmatched = true;
              return Result<EndpointPlan>::Ok(std::move(plan));
            }
          }
        } else if (plan.connections.empty()) {
          plan.connections = ProcedureConnector::BuildSidSpliceDeparture(*cifp, apt, builder, rwy);
        }
        plan.used_procedures = !plan.connections.empty();
      } else {
        plan.connections = ProcedureConnector::BuildArrival(*cifp, apt, builder, rwy);
        // Named --star must match a STAR connection (on-net or splice); never
        // silently fall through to approach IAFs (D1 / #30).
        if (!sel.empty()) {
          if (!FilterConnectionsByName(plan.connections, sel)) {
            plan.connections =
                ProcedureConnector::BuildStarSpliceArrival(*cifp, apt, builder, rwy, sel);
            if (plan.connections.empty()) {
              plan.named_procedure_unmatched = true;
              return Result<EndpointPlan>::Ok(std::move(plan));
            }
          }
          plan.used_procedures = true;
        } else if (!plan.connections.empty()) {
          // On-network published STAR gates — today's path; do not mix approach.
          plan.used_procedures = true;
        } else {
          // Off-network published gates (or no STAR): STAR splice ∪ approach
          // in one pool. Soft-prefer STAR only when splice candidates exist.
          std::vector<Connection> pool =
              ProcedureConnector::BuildStarSpliceArrival(*cifp, apt, builder, rwy);
          std::vector<Connection> apch =
              ProcedureConnector::BuildApproachArrival(*cifp, apt, builder, rwy);
          const bool soft_prefer = !pool.empty() && !apch.empty();
          // STAR first, then approach: equal effective seeds keep STAR at front.
          MergeAll(pool, std::move(apch), soft_prefer);
          SortConnectionsBySeed(pool, soft_prefer);
          plan.connections = std::move(pool);
          // Plan-level flags are only meaningful for homogeneous pools; mixed
          // STAR∪approach is classified per path from procedures.front().type.
          bool any_star = false;
          bool any_apch = false;
          for (const Connection& c : plan.connections) {
            for (const ProcedureRef& ref : c.procedures) {
              if (ref.type == ProcedureType::kApproach) {
                any_apch = true;
              } else if (ref.type == ProcedureType::kStar) {
                any_star = true;
              }
            }
          }
          plan.used_procedures = any_star && !any_apch;
          plan.used_approach = any_apch && !any_star;
        }
      }
    } else if (!(departure ? request.departure_sid : request.arrival_star).empty()) {
      // A procedure was named but the airport has no CIFP data at all.
      plan.named_procedure_unmatched = true;
      return Result<EndpointPlan>::Ok(std::move(plan));
    }
    if (plan.connections.empty()) {
      // No usable procedures: fall back to DCT links to the nearest
      // on-network waypoints. The airport stays the route endpoint; the
      // connecting leg shows "DCT" since no procedure was selected. Filter by
      // direction: a departure needs an outbound-capable fix, an arrival an
      // inbound-capable one (a STAR entry gate is often inbound-only).
      plan.connections =
          ProcedureConnector::BuildDctFallback(apt, builder, 5, /*arrival=*/!departure);
    }
    return Result<EndpointPlan>::Ok(std::move(plan));
  }
  // Not an airport ICAO: leave connections empty; the caller reports unknown
  // departure/arrival. Waypoint / IDENT/REGION endpoints are intentionally
  // unsupported (idents are not globally unique).
  return Result<EndpointPlan>::Ok(std::move(plan));
}

}  // namespace bf
