// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "core/result.h"
#include "io/build/procedure_connector.h"

namespace bf {

class GraphBuilder;
struct CifpData;
struct RouteRequest;

// How one endpoint of a query attaches to the enroute graph. An airport with
// procedures contributes several seeded connection fixes; a DCT-fallback
// airport contributes one or a few. `airport_icao` is always set for a
// successful plan (FindRoutes only accepts airport ICAO endpoints).
// `has_procedures` records whether the airport actually publishes procedures
// for this side (SID for departure; STAR or approach for arrival), so a DCT
// fallback can be told apart from missing data: procedures that exist but
// reach no on-network fix (radar vectors) still fall back to DCT.
struct EndpointPlan {
  std::vector<Connection> connections;
  std::string airport_icao;  // empty if the endpoint is a plain waypoint
  bool used_procedures = false;
  bool used_approach = false;
  bool has_procedures = false;
  // Set when the request named a SID/STAR that the airport does not publish (or
  // whose fixes reach no on-network vertex): the caller reports an Error instead
  // of silently falling back to DCT or another procedure.
  bool named_procedure_unmatched = false;
};

// Injected CIFP lookup so EndpointPlanner never touches NavDatabase's private
// procedure cache (ProceduresFor stays private on NavDatabase).
using CifpLookup = std::function<Result<const CifpData*>(const std::string& icao)>;

// Plans airport → enroute connection fixes (SID/STAR/splice/DCT) for one
// FindRoutes endpoint. Stateless; all entry points are static.
class EndpointPlanner {
 public:
  EndpointPlanner() = delete;

  // Soft-prefer STAR splice over approach when both compete in the same arrival
  // pool (issue #30). Applied to search ranking / SeededEndpoint.cost only —
  // never written into Connection::seed_distance_nm (which stays geographic for
  // MakeRoute and reported distances). Pure no-STAR (#24) airports must not pay
  // this bump.
  static constexpr double kProcedurePreferNm = 15.0;

  // Resolve an airport endpoint into how it attaches to the network. With CIFP
  // procedures: procedure-first connections; without CIFP: DCT links to the
  // nearest on-network waypoints. Non-airport tokens leave connections empty so
  // the caller reports an unknown-airport error (waypoint endpoints are not
  // supported — idents are not globally unique).
  static Result<EndpointPlan> Plan(const GraphBuilder& builder, const RouteRequest& request,
                                   const std::string& name, bool departure,
                                   const CifpLookup& cifp_lookup);

  // Convert connections to A* seeded endpoints. When soft_prefer_star is true,
  // approach-front connections pay kProcedurePreferNm on SeededEndpoint.cost
  // only (stored seed_distance_nm stays geographic).
  static std::vector<SeededEndpoint> ToSearchEndpoints(const std::vector<Connection>& connections,
                                                       bool soft_prefer_star);

  // True when the arrival pool still holds both STAR and approach refs
  // (soft-prefer was armed at build time and both sides survived Accumulate).
  static bool SoftPreferStarActive(const EndpointPlan& plan);
};

}  // namespace bf
