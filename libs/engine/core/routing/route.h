// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/domain/coordinate.h"

namespace bf {

// How one endpoint of a route attaches to the enroute network.
enum class ConnectionKind {
  kProcedure,     // via a named SID/STAR (the sid/star field holds its name)
  kDirect,        // DCT fallback: the airport has no procedure data for this side
  kRadarVectors,  // procedures exist but none reach an on-network fix (a radar-
                  // vectored departure/arrival); the search fell back to a DCT
                  // link, but this is a real procedure situation, not missing data
};

// The stable token form of a connection kind, as used in JSON output and any
// other machine-readable representation.
inline const char* ToString(ConnectionKind k) {
  switch (k) {
    case ConnectionKind::kProcedure:
      return "procedure";
    case ConnectionKind::kDirect:
      return "direct";
    case ConnectionKind::kRadarVectors:
      return "radar_vectors";
  }
  return "direct";
}

// A single leg of a computed route: a segment from one point to the next. `via`
// names the ATS airway designator, "DCT" for a direct leg, or the literal
// connector keyword "SID"/"STAR" for the airport<->network procedure legs (the
// procedure names themselves live in Route::sid/star, not here).
struct RouteLeg {
  std::string from{};
  std::string to{};
  std::string via{};  // airway designator, "DCT", or "SID"/"STAR" for procedure legs
  double distance_nm = 0.0;

  // When the underlying airway segment is a concurrency (two or more named
  // airways sharing this physical leg, encoded "A593-Y592" in the source data),
  // this holds every designator on the leg. `via` then carries just the one
  // chosen for the filed route. Empty for an ordinary single-airway or DCT leg.
  std::vector<std::string> concurrent_airways;
};

// A point along a computed route, for display / export.
struct RoutePoint {
  std::string ident{};
  Coordinate coord{};
};

// A computed route between two endpoints.
struct Route {
  std::vector<RoutePoint> points;
  std::vector<RouteLeg> legs;
  double total_distance_nm = 0.0;
  std::string route_string;  // filed-flight-plan form "DEP SID FIX AWY FIX STAR ARR"

  // The total distance split by flight phase, filled at construction so display
  // never re-derives it. `dep`/`arr` are the departure/arrival procedure-leg
  // (airport <-> connection fix) distances, 0 when the endpoint is a plain
  // waypoint; `enroute` is everything between (total - dep - arr). Their sum
  // equals total_distance_nm.
  double dep_distance_nm = 0.0;
  double enroute_distance_nm = 0.0;
  double arr_distance_nm = 0.0;

  // Terminal procedures used to connect the airports to the enroute network.
  // Empty when an endpoint is a plain waypoint or fell back to a DCT link.
  std::string sid;         // departure SID name, e.g. "DEEZZ5"
  std::string star;        // arrival STAR name, e.g. "CAMRN5"
  std::string dep_runway;  // departure runway if known, e.g. "RW31L"
  std::string arr_runway;  // arrival runway if known

  // How each endpoint attaches to the network. kProcedure when a SID/STAR was
  // selected (sid/star names it); kRadarVectors when procedures exist but none
  // reach an on-network fix (radar vectors, fell back to DCT); kDirect when the
  // airport has no procedure data. Symmetric across departure/arrival even
  // though today only departures see kRadarVectors in practice.
  ConnectionKind dep_connection = ConnectionKind::kDirect;
  ConnectionKind arr_connection = ConnectionKind::kDirect;

  // All SID/STAR(+runway) combinations that share the chosen connection fix and
  // are therefore interchangeable for this route, formatted "NAME.TRANSITION".
  // The route was computed once for the shared fix rather than per procedure.
  std::vector<std::string> sid_options;
  std::vector<std::string> star_options;

  // The forced ("via") points the route was routed through, in order, echoed as
  // resolved "IDENT/REGION" keys. This makes the disambiguation visible when a
  // bare ident matched several regions. Empty when no forced points were given.
  std::vector<std::string> forced_points;
};

}  // namespace bf
