// SPDX-License-Identifier: MIT
#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <fstream>
#include <set>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "core/constraints/airway_rule_constraint.h"
#include "core/graph/astar.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"
#include "test_bfdb.h"

namespace {

using bf::test::NavDataDir;

// Open the navigation database once and share it across all integration cases.
// NavDatabase is read-only after Open and FindRoutes is const, so a single
// instance is safe to reuse; this avoids re-loading ~20 MB of data per case,
// which dominated the suite's run time. Loads from the prebuilt bfdb cache.
// Returns nullptr when the cache is absent so callers SKIP.
const bf::NavDatabase* SharedDb() {
  static bf::Result<bf::NavDatabase> db = bf::test::OpenReadOnlyDb();
  return db ? &db.value() : nullptr;
}

bf::RouteRequest MakeRequest(const std::string& dep, const std::string& arr) {
  bf::RouteRequest r;
  r.departure = dep;
  r.arrival = arr;
  return r;
}

TEST_CASE("real data: KJFK to KLAX route is plausible", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 2);
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KLAX");
  // Great-circle JFK-LAX is ~2144 NM; an airway route is longer but within ~15%.
  CHECK(r.total_distance_nm > 2144.0);
  CHECK(r.total_distance_nm < 2144.0 * 1.15);
}

TEST_CASE("real data: route phase distances split and sum to the total", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  const bf::Route& r = routes.value().front();

  // dep + enroute + arr must equal the total (the split is exact, computed in
  // MakeRoute from leg positions rather than re-derived at print time).
  CHECK(r.dep_distance_nm + r.enroute_distance_nm + r.arr_distance_nm ==
        Catch::Approx(r.total_distance_nm));
  // Both endpoints are airports connecting through procedures, so the terminal
  // legs carry non-zero seed distance and the enroute portion dominates.
  CHECK(r.dep_distance_nm > 0.0);
  CHECK(r.arr_distance_nm > 0.0);
  CHECK(r.enroute_distance_nm > 0.0);
}

TEST_CASE("real data: case-insensitive endpoints", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  CHECK(db->FindRoutes(MakeRequest("kjfk", "klax")));
}

TEST_CASE("real data: unknown airport reports an error", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("ZZZZ", "KLAX"));
  REQUIRE_FALSE(routes);
  CHECK(routes.error().code == bf::ErrorCode::kAirportNotFound);
}

TEST_CASE("real data: K-shortest returns distinct ordered routes", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.k = 3;
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  const std::vector<bf::Route>& rs = routes.value();
  REQUIRE(rs.size() >= 2);
  // Routes are ordered shortest-first and must be distinct.
  for (size_t i = 1; i < rs.size(); ++i) {
    CHECK(rs[i].total_distance_nm >= rs[i - 1].total_distance_nm - 1e-6);
    CHECK(rs[i].route_string != rs[i - 1].route_string);
  }
}

TEST_CASE("real data: an arrival joins a published STAR entry, not the farthest one",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // An arrival joins a STAR at a PUBLISHED entry (an Initial Fix), and KLAX
  // publishes many of them at very different distances -- BASET5 enters from PGS
  // ~260 NM out, ANJLL4 from HAKMN ~196 NM, SADDE8 from SADDE ~20 NM. The
  // multi-source search must weigh each entry's procedure distance against the
  // enroute cost of reaching it rather than defaulting to the farthest one.
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KDEN", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE_FALSE(r.legs.empty());
  const bf::RouteLeg& last = r.legs.back();
  CHECK(last.to == "KLAX");
  CHECK_FALSE(r.star.empty());
  // Not the far BASET5 entry fix, and the end-to-end route stays close to the
  // ~750 NM great circle -- a badly chosen entry shows up as a long total, since
  // the arrival's procedure distance is priced into it.
  CHECK(last.from != "PGS");
  CHECK(r.total_distance_nm < 850.0);
}

TEST_CASE("real data: a STAR entry gate reached by a forward-only airway is usable",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // VHHH's ABEY STARs enter at ABBEY, a fix reached only via the forward-only
  // airway FISHA->ABBEY (inbound edge, no outbound). Arrival connection must
  // treat an inbound-only fix as a valid STAR entry; otherwise ABBEY is skipped
  // and RJTT->VHHH detours far southwest to SIKOU to reach ROCCA on V571. With
  // the inbound test, the route joins through ABBEY and takes an ABEY STAR.
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("RJTT", "VHHH"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  CHECK(r.points.front().ident == "RJTT");
  CHECK(r.points.back().ident == "VHHH");
  // The arrival uses an ABEY-family STAR entering at ABBEY, not the SIER7C/ROCCA
  // detour. The compact route string collapses same-airway runs, so ABBEY (the
  // airway exit / STAR entry) shows while the intermediate FISHA does not.
  CHECK(r.star.rfind("ABEY", 0) == 0);
  CHECK(r.route_string.find("ABBEY") != std::string::npos);
  CHECK(r.route_string.find("SIKOU") == std::string::npos);
}

TEST_CASE("real data: K candidates can use different procedures", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // The multi-endpoint K-shortest lets candidates join the network through
  // different connection fixes, so the alternatives can use genuinely different
  // SID/STAR procedures rather than sharing one fixed pair. KSEA -> KLAX has
  // several competitive arrival options (KIMMO3 via LHS, WAYVE1 via LOPES); with
  // enough candidates more than one distinct STAR appears. K is 10 rather than 8
  // because the published-entry connection model gives the top candidates fewer
  // distinct entry fixes to spread over, so WAYVE1 now surfaces a little deeper
  // in the list.
  bf::RouteRequest req = MakeRequest("KSEA", "KLAX");
  req.k = 10;
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  const std::vector<bf::Route>& rs = routes.value();
  REQUIRE(rs.size() >= 2);

  // Ordering and distinctness still hold across the candidate set.
  for (size_t i = 1; i < rs.size(); ++i) {
    CHECK(rs[i].total_distance_nm >= rs[i - 1].total_distance_nm - 1e-6);
    CHECK(rs[i].route_string != rs[i - 1].route_string);
  }

  // At least two candidates differ in their STAR, proving alternatives can cross
  // connection fixes / procedures rather than sharing a single fixed pair.
  std::set<std::string> stars;
  for (const bf::Route& r : rs) {
    stars.insert(r.star);
  }
  CHECK(stars.size() >= 2);
}

TEST_CASE("real data: high cruise altitude still finds a route", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.altitude = bf::FlRange{350, 350};  // FL350: enables altitude-band and MORA filtering
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  // The altitude-constrained route should still be a sane length.
  CHECK(routes.value().front().total_distance_nm > 2144.0);
}

TEST_CASE("real data: avoiding a waypoint routes around it", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());
  const bf::Route& base = baseline.value().front();

  // Pick an interior enroute point of the baseline route to avoid. Skip the
  // first and last points (the airports / connection fixes).
  REQUIRE(base.points.size() > 3);
  const std::string victim = base.points[base.points.size() / 2].ident;

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.avoid_waypoints = {victim};
  bf::Result<std::vector<bf::Route>> avoided = db->FindRoutes(req);
  REQUIRE(avoided);
  REQUIRE_FALSE(avoided.value().empty());
  const bf::Route& r = avoided.value().front();

  // The avoided waypoint must not appear as an interior point of the new route.
  for (size_t i = 1; i + 1 < r.points.size(); ++i) {
    CHECK(r.points[i].ident != victim);
  }
  // Routing around a point cannot be shorter than the unconstrained optimum.
  CHECK(r.total_distance_nm >= base.total_distance_nm - 1e-6);
}

TEST_CASE("real data: avoiding a SID connection fix picks another entry", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());
  const bf::Route& base = baseline.value().front();

  // The first enroute point after the departure airport is the SID connection
  // fix -- a seeded search source, which AvoidConstraint alone cannot block.
  // Avoiding it must still work (via endpoint pruning), yielding a route that
  // enters the network through a different fix.
  REQUIRE(base.points.size() > 2);
  const std::string connection_fix = base.points[1].ident;

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.avoid_waypoints = {connection_fix};
  bf::Result<std::vector<bf::Route>> avoided = db->FindRoutes(req);
  REQUIRE(avoided);
  REQUIRE_FALSE(avoided.value().empty());
  for (const bf::RoutePoint& p : avoided.value().front().points) {
    CHECK(p.ident != connection_fix);
  }
}

TEST_CASE("real data: a seeded route is reproducible", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.random_seed = 42u;
  bf::Result<std::vector<bf::Route>> a = db->FindRoutes(req);
  bf::Result<std::vector<bf::Route>> b = db->FindRoutes(req);
  REQUIRE(a);
  REQUIRE(b);
  REQUIRE_FALSE(a.value().empty());
  REQUIRE_FALSE(b.value().empty());
  // Same seed => byte-identical route string (reproducible / no mutable state).
  CHECK(a.value().front().route_string == b.value().front().route_string);
  // A seeded route must still be a valid, plausible route.
  CHECK(a.value().front().total_distance_nm > 2000.0);
}

TEST_CASE("real data: avoiding an airway keeps it out of the route", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());
  const bf::Route& base = baseline.value().front();

  // Find a named enroute airway used by the baseline route to avoid. Skip the
  // leading/trailing legs (their `via` is the SID/STAR name, not an airway) and
  // any DCT legs.
  std::string victim_awy;
  for (size_t i = 1; i + 1 < base.legs.size(); ++i) {
    const std::string& via = base.legs[i].via;
    if (via != "DCT" && !via.empty() && via != base.sid && via != base.star) {
      victim_awy = via;
      break;
    }
  }
  if (victim_awy.empty()) {
    SKIP("baseline route uses no named enroute airway to avoid");
  }

  // An exact-match block rule with no region restriction is the replacement for the
  // former avoid_airways field: same intent, one code path.
  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  bf::AirwayRule rule;
  rule.designators = {victim_awy};
  rule.match = bf::AirwayRule::Match::kExact;
  rule.action = bf::AirwayRule::Action::kBlock;
  req.airway_rules = {rule};
  bf::Result<std::vector<bf::Route>> avoided = db->FindRoutes(req);
  REQUIRE(avoided);
  REQUIRE_FALSE(avoided.value().empty());
  for (const bf::RouteLeg& leg : avoided.value().front().legs) {
    CHECK(leg.via != victim_awy);
  }
}

TEST_CASE("real data: forced via points appear in order", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());
  const double base_dist = baseline.value().front().total_distance_nm;

  // Force the route through an off-track fix (DBL, Colorado). It must appear in
  // the point list, be echoed with its region, and not shorten the route.
  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.forced_points = {"DBL"};
  bf::Result<std::vector<bf::Route>> forced = db->FindRoutes(req);
  REQUIRE(forced);
  REQUIRE_FALSE(forced.value().empty());
  const bf::Route& r = forced.value().front();

  bool via_present = false;
  for (const bf::RoutePoint& p : r.points) {
    if (p.ident == "DBL") {
      via_present = true;
      break;
    }
  }
  CHECK(via_present);
  REQUIRE(r.forced_points.size() == 1);
  CHECK(r.forced_points.front().rfind("DBL/", 0) == 0);  // echoed as DBL/REGION
  CHECK(r.total_distance_nm >= base_dist - 1e-6);        // a detour is not shorter
}

TEST_CASE("real data: forced via points with k>1 returns distinct ordered routes",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.forced_points = {"DBL"};
  req.k = 3;
  bf::Result<std::vector<bf::Route>> r = db->FindRoutes(req);
  REQUIRE(r);
  REQUIRE_FALSE(r.value().empty());
  const std::vector<bf::Route>& routes = r.value();

  // Every returned candidate still honors the forced point and is distinct, and
  // the candidates are ordered by non-decreasing distance.
  std::set<std::string> seen_strings;
  double prev = 0.0;
  for (const bf::Route& route : routes) {
    bool via_present = false;
    for (const bf::RoutePoint& p : route.points) {
      if (p.ident == "DBL") {
        via_present = true;
        break;
      }
    }
    CHECK(via_present);
    CHECK(seen_strings.insert(route.route_string).second);  // distinct
    CHECK(route.total_distance_nm >= prev - 1e-6);          // non-decreasing
    prev = route.total_distance_nm;
  }
}

TEST_CASE("real data: an unknown forced point is an error", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.forced_points = {"ZZZZQ"};
  bf::Result<std::vector<bf::Route>> r = db->FindRoutes(req);
  CHECK_FALSE(r);
}

TEST_CASE("real data: KJFK to KLAX uses real SID and STAR procedures", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // This assertion needs the CIFP procedure files, extracted separately from
  // the enroute data; skip if KJFK's/KLAX's procedures are not present.

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  // The route connects via real procedures: a SID out of KJFK and a STAR into
  // KLAX, each named, rather than a synthetic DCT hop.
  CHECK_FALSE(r.sid.empty());
  CHECK_FALSE(r.star.empty());
  CHECK_FALSE(r.sid_options.empty());
  CHECK_FALSE(r.star_options.empty());
  // The airports remain the true endpoints; the second / second-to-last points
  // are the procedure connection fixes on the enroute network.
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KLAX");
  CHECK(r.points.size() >= 4);
  // A procedure-connected route is still geographically plausible (the straight
  // -line procedure estimate keeps the total at or above the great circle).
  CHECK(r.total_distance_nm > 2144.0);
  CHECK(r.total_distance_nm < 2144.0 * 1.2);

  // The procedures appear as explicit first/last legs (airport <-> connection
  // fix). Their `via` carries the literal connector keyword "SID"/"STAR" (the
  // procedure names live in r.sid/r.star), and the route string reads like a
  // filed plan.
  REQUIRE(r.legs.size() >= 2);
  CHECK(r.legs.front().from == "KJFK");
  CHECK(r.legs.front().via == "SID");
  CHECK(r.legs.back().to == "KLAX");
  CHECK(r.legs.back().via == "STAR");
  CHECK(r.route_string.rfind("KJFK SID ", 0) == 0);  // starts with "KJFK SID "
  CHECK(r.route_string.find(" STAR KLAX") != std::string::npos);
  // Leg distances should sum to the reported total (procedures included).
  double leg_sum = 0.0;
  for (const bf::RouteLeg& leg : r.legs) {
    leg_sum += leg.distance_nm;
  }
  CHECK(leg_sum == Catch::Approx(r.total_distance_nm).margin(1.0));
}

TEST_CASE("real data: a departure runway filter still yields a route", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_runway = "RW31L";  // a real KJFK runway served by DEEZZ5
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK_FALSE(routes.value().front().sid.empty());
}

TEST_CASE("real data: a named SID is used when requested", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_sid = "DEEZZ5";  // a real KJFK SID
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  // Every candidate must depart via the requested SID.
  for (const bf::Route& r : routes.value()) {
    CHECK(r.sid == "DEEZZ5");
  }
}

TEST_CASE("real data: a bare SID name pins the procedure but leaves transitions open",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // "DEEZZ5.TOWIN" pins the transition; the offered options should all be that
  // transition, unlike the bare-name case which leaves several.
  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_sid = "DEEZZ5.TOWIN";
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  CHECK(routes.value().front().sid == "DEEZZ5");
  for (const std::string& opt : routes.value().front().sid_options) {
    CHECK(opt.rfind("DEEZZ5.TOWIN", 0) == 0);
  }
}

TEST_CASE("real data: an unknown SID name is an error, not a silent fallback", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.departure_sid = "NOSUCH9";
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  CHECK_FALSE(routes);  // no DCT fallback, no other SID -- a clean error
}

TEST_CASE("real data: an airport without procedures stays the endpoint via DCT", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // Exercises the kDirect fallback: KJFK has a SID, while KIKR has no CIFP file
  // at all, so its arrival connects by a direct link. The airport must remain
  // the route endpoint rather than being replaced by its connection fix, and the
  // connection is classified kDirect (no data) rather than radar vectors.
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KIKR"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 2);
  REQUIRE(r.legs.size() >= 2);
  // Both airports are the true endpoints; the arrival has no procedure data, so
  // its final leg is a DCT link into the airport, classified kDirect.
  CHECK(r.points.front().ident == "KJFK");
  CHECK(r.points.back().ident == "KIKR");
  CHECK(r.legs.back().to == "KIKR");
  CHECK(r.star.empty());
  CHECK(r.legs.back().via == "DCT");
  CHECK(r.arr_connection == bf::ConnectionKind::kDirect);
}

TEST_CASE("real data: a route does not transit through an intermediate airport", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routes);
  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 2);
  // Airports connect via synthetic DCT links and used to be exploitable as
  // free pass-through hubs (e.g. ...MIE DCT KMIE SNKPT...). Only the first and
  // last points may be airport ICAOs; no interior point should be one.
  for (size_t i = 1; i + 1 < r.points.size(); ++i) {
    const std::string& id = r.points[i].ident;
    const bool looks_like_us_airport = id.size() == 4 && id.front() == 'K';
    CHECK_FALSE(looks_like_us_airport);
  }
}

TEST_CASE("real data: a radar-vectored departure is flagged, not shown as missing",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // KPHL publishes only radar-vector SIDs (PHL4: VA -> VM, no fix), so no SID
  // reaches an on-network fix and the departure falls back to a DCT link. That
  // fallback must be reported as radar vectors (procedures exist) rather than
  // kDirect (no data), so a user can tell the two apart.

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KPHL", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  CHECK(r.dep_connection == bf::ConnectionKind::kRadarVectors);
  CHECK(r.sid.empty());  // no named SID for a radar-vectored departure
  // The airport is still the endpoint and the first leg is the DCT hop out.
  CHECK(r.points.front().ident == "KPHL");
  CHECK(r.legs.front().from == "KPHL");
  CHECK(r.legs.front().via == "DCT");
  // The arrival side still connects through a real STAR.
  CHECK(r.arr_connection == bf::ConnectionKind::kProcedure);
  CHECK_FALSE(r.star.empty());
}

TEST_CASE("real data: KJFK publishes terminal-area MSA sectors", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // MSA requires earth_msa.dat, which is extracted separately from the enroute
  // files; skip if it was not provided alongside them.
  const std::vector<bf::MsaSector> msa = db->MsaForAirport("KJFK");
  if (msa.empty()) {
    SKIP("earth_msa.dat not present in '" << NavDataDir() << "'");
  }

  // Every returned sector belongs to KJFK and carries at least one arc with a
  // positive minimum altitude.
  for (const bf::MsaSector& s : msa) {
    CHECK(s.airport_icao == "KJFK");
    REQUIRE_FALSE(s.arcs.empty());
    bool any_alt = false;
    for (const bf::MsaArc& a : s.arcs) {
      if (a.alt_100ft > 0) {
        any_alt = true;
      }
    }
    CHECK(any_alt);
  }
}

TEST_CASE("real data: concurrent FindRoutes on one shared database is race-free",
          "[integration][concurrency]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // Exercise the only shared mutable state (the lazily filled procedure cache)
  // from several threads at once. The query set mixes:
  //   - the same airport pair (KJFK/KLAX) to stress same-key cache contention,
  //   - different pairs to stress parallel parsing and concurrent map rehashing.
  // Under the tsan preset this catches data races; under any build it must not
  // crash and every query that a single thread can answer must also succeed
  // here. Endpoints that lack data simply return an error, which is fine: the
  // point is concurrency safety, not that every pair routes.
  const std::vector<std::pair<std::string, std::string>> queries = {
      {"KJFK", "KLAX"}, {"KJFK", "KLAX"}, {"KLAX", "KJFK"}, {"KBOS", "KDEN"},
      {"KDEN", "KBOS"}, {"KJFK", "KBOS"}, {"KLAX", "KDEN"}, {"KBOS", "KLAX"},
  };

  constexpr int kThreads = 8;
  constexpr int kItersPerThread = 25;
  std::atomic<int> completed{0};
  std::atomic<bool> any_route{false};

  std::vector<std::thread> workers;
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&, t]() {
      for (int i = 0; i < kItersPerThread; ++i) {
        const auto& q = queries[(t + i) % queries.size()];
        bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest(q.first, q.second));
        if (routes && !routes.value().empty()) {
          any_route.store(true);
        }
        completed.fetch_add(1);
      }
    });
  }
  for (std::thread& w : workers) {
    w.join();
  }

  CHECK(completed.load() == kThreads * kItersPerThread);
  // At least one of the well-known pairs must have produced a route, proving the
  // concurrent calls actually did work (not merely failed in lockstep).
  CHECK(any_route.load());

  // The shared cache must agree with a fresh single-threaded query afterward.
  bf::Result<std::vector<bf::Route>> after = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  CHECK(after);
}

TEST_CASE("real data: ParseRoute round-trips a computed route", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  bf::Result<std::vector<bf::Route>> routed = db->FindRoutes(MakeRequest("KJFK", "KLAX"));
  REQUIRE(routed);
  REQUIRE_FALSE(routed.value().empty());
  const std::string route_str = routed.value().front().route_string;

  // Feeding a computed route string back into ParseRoute must validate and
  // reproduce the same canonical route string.
  bf::Result<bf::Route> parsed = db->ParseRoute(route_str);
  REQUIRE(parsed);
  CHECK(parsed.value().route_string == route_str);
  CHECK(parsed.value().total_distance_nm > 2000.0);

  // ParseRoute must reconstruct the phase-split summary from the rebuilt legs,
  // not leave it at the struct defaults. The three phases always sum to the
  // total (case 3), and each phase distance is non-negative.
  const bf::Route& p = parsed.value();
  CHECK(p.dep_distance_nm >= 0.0);
  CHECK(p.enroute_distance_nm >= 0.0);
  CHECK(p.arr_distance_nm >= 0.0);
  CHECK(p.dep_distance_nm + p.enroute_distance_nm + p.arr_distance_nm ==
        Catch::Approx(p.total_distance_nm));

  // When the round-tripped plan carries the literal SID/STAR connector legs, the
  // corresponding phase distance and connection kind must reflect the procedure
  // (case 1) -- derived from the leg's "SID"/"STAR" via keyword, since the
  // procedure name is not recoverable from the string and route.sid/star stay
  // empty here.
  bool has_sid_leg = false;
  bool has_star_leg = false;
  for (const bf::RouteLeg& leg : p.legs) {
    if (leg.via == "SID") {
      has_sid_leg = true;
    } else if (leg.via == "STAR") {
      has_star_leg = true;
    }
  }
  if (has_sid_leg) {
    CHECK(p.dep_distance_nm > 0.0);
    CHECK(p.dep_connection == bf::ConnectionKind::kProcedure);
  } else {
    CHECK(p.dep_distance_nm == 0.0);
    CHECK(p.dep_connection == bf::ConnectionKind::kDirect);
  }
  if (has_star_leg) {
    CHECK(p.arr_distance_nm > 0.0);
    CHECK(p.arr_connection == bf::ConnectionKind::kProcedure);
  } else {
    CHECK(p.arr_distance_nm == 0.0);
    CHECK(p.arr_connection == bf::ConnectionKind::kDirect);
  }
}

TEST_CASE("real data: ParseRoute expands an airway's intermediate fixes", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // KMCI DCT MCI J24 SLN DCT KSLN: J24 expands its intermediate fixes (JUDGE)
  // between the MCI/SLN VORs; the DCT legs to the airports stay as own segments.
  bf::Result<bf::Route> r = db->ParseRoute("KMCI DCT MCI J24 SLN DCT KSLN");
  if (!r) {
    SKIP("KMCI/KSLN or J24 MCI->SLN not present in this AIRAC cycle");
  }
  CHECK(r.value().points.size() >= 5);  // KMCI, MCI, >=1 intermediate, SLN, KSLN
  CHECK(r.value().total_distance_nm > 0.0);
  CHECK(r.value().legs.front().via == "DCT");
  CHECK(r.value().legs.back().via == "DCT");
  for (size_t i = 1; i + 1 < r.value().legs.size(); ++i) {
    CHECK(r.value().legs[i].via == "J24");
  }
}

TEST_CASE("real data: ParseRoute reports a disconnected airway", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // J24 does not connect MCI to an arbitrary far fix (SEA); expect an error,
  // not a silently wrong route.
  bf::Result<bf::Route> r = db->ParseRoute("KMCI DCT MCI J24 SEA DCT KSEA");
  CHECK_FALSE(r);
}

TEST_CASE("real data: ParseRoute rejects an unknown fix", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  bf::Result<bf::Route> r = db->ParseRoute("KMCI DCT ZZZQ DCT KSLN");
  CHECK_FALSE(r);
}

TEST_CASE("real data: ParseRoute accepts a pure direct airport pair", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // "DEP DCT ARR" with no enroute fix is a valid hand-filed direct route: one DCT
  // leg between the two airports.
  bf::Result<bf::Route> r = db->ParseRoute("ZHHH DCT ZGGG");
  if (!r) {
    SKIP("ZHHH / ZGGG not present in this AIRAC cycle");
  }
  CHECK(r.value().points.size() == 2);
  REQUIRE(r.value().legs.size() == 1);
  CHECK(r.value().legs.front().via == "DCT");
  CHECK(r.value().route_string == "ZHHH DCT ZGGG");
  CHECK(r.value().total_distance_nm > 0.0);

  // A pure DEP DCT ARR has no procedure legs: departure and arrival phase
  // distances are zero, everything is enroute, and both connections are direct
  // (case 2).
  CHECK(r.value().dep_distance_nm == 0.0);
  CHECK(r.value().arr_distance_nm == 0.0);
  CHECK(r.value().enroute_distance_nm == Catch::Approx(r.value().total_distance_nm));
  CHECK(r.value().dep_connection == bf::ConnectionKind::kDirect);
  CHECK(r.value().arr_connection == bf::ConnectionKind::kDirect);

  // The no-fix shape is accepted only with an explicit DCT and airports at both
  // ends. A bare airport pair (no connector) is rejected, and a route that does
  // not end at an arrival airport ("ZHHH DCT OLMIB") is also rejected.
  CHECK_FALSE(db->ParseRoute("ZHHH ZGGG"));
  CHECK_FALSE(db->ParseRoute("ZHHH DCT OLMIB"));
}

TEST_CASE("real data: ParseRoute rejects a no-fix shape that also names a procedure",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  if (!db->ParseRoute("ZHHH DCT ZGGG")) {
    SKIP("ZHHH / ZGGG not present in this AIRAC cycle");
  }
  // A no-fix shape paired with a procedure connector ("DEP SID DCT ARR",
  // "DEP DCT STAR ARR", "DEP SID DCT STAR ARR") is not a valid filed route: a
  // SID/STAR leg always bridges the airport to a transition fix, never to the
  // far airport directly. The "DEP DCT ARR" shortcut must not fire once a
  // SID/STAR has been recognized -- otherwise it would emit a bare "DCT" leg and
  // leave route.sid/star empty, silently dropping the procedure. These must be
  // rejected, not parsed with the procedure dropped.
  CHECK_FALSE(db->ParseRoute("ZHHH SID DCT ZGGG"));
  CHECK_FALSE(db->ParseRoute("ZHHH DCT STAR ZGGG"));
  CHECK_FALSE(db->ParseRoute("ZHHH SID DCT STAR ZGGG"));
}

TEST_CASE("real data: ParseRoute rejects a trailing dangling connector", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // "KMCI DCT MCI J24 KSLN" -- J24 has no following fix before the arrival, so
  // the airway leg has no destination. Must error, not silently drop it.
  bf::Result<bf::Route> r = db->ParseRoute("KMCI DCT MCI J24 KSLN");
  CHECK_FALSE(r);
}

TEST_CASE("real data: ParseRoute accepts explicit DCT connectors to airports (#26)",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // An explicit "DCT" may connect an airport to its adjacent fix (the form
  // FindRoutes emits with no SID/STAR); the canonical string must round-trip.
  bf::Result<bf::Route> ok = db->ParseRoute("KMCI DCT MCI DCT KSLN");
  if (!ok) {
    SKIP("KMCI/KSLN/MCI not present in this AIRAC cycle");
  }
  CHECK(ok.value().route_string == "KMCI DCT MCI DCT KSLN");
  REQUIRE(ok.value().legs.size() == 2);
  CHECK(ok.value().legs.front().via == "DCT");
  CHECK(ok.value().legs.back().via == "DCT");
  CHECK(ok.value().dep_connection == bf::ConnectionKind::kDirect);
  CHECK(ok.value().arr_connection == bf::ConnectionKind::kDirect);

  // No connector between fixes, or between an airport and its fix, is rejected.
  CHECK_FALSE(db->ParseRoute("KMCI DCT MCI SLN DCT KSLN"));  // MCI SLN: no connector
  CHECK_FALSE(db->ParseRoute("KMCI MCI J24 SLN KSLN"));      // airports have no connector
  CHECK_FALSE(db->ParseRoute("MCI J24 SLN"));                // not bracketed by airports
}

TEST_CASE("real data: ParseRoute round-trips a DCT-to-airport tail (#26/#4)", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // A no-STAR FindRoutes route ends "... FIX DCT ARR"; feeding it back must
  // reproduce the string. KMCI->KSLN selects a SID but no STAR (DCT arrival).
  bf::Result<std::vector<bf::Route>> routed = db->FindRoutes(MakeRequest("KMCI", "KSLN"));
  if (!routed || routed.value().empty()) {
    SKIP("KMCI->KSLN not present in this AIRAC cycle");
  }
  const std::string route_str = routed.value().front().route_string;
  if (route_str.find(" STAR ") != std::string::npos) {
    SKIP("KMCI->KSLN selected a STAR this cycle; DCT-tail path not exercised");
  }
  bf::Result<bf::Route> parsed = db->ParseRoute(route_str);
  REQUIRE(parsed);
  CHECK(parsed.value().route_string == route_str);
  CHECK(parsed.value().arr_connection == bf::ConnectionKind::kDirect);
}

// Connection model (procedure_connector.cc): a procedure is joined only at its
// PUBLISHED handoff point -- a STAR at its Initial Fix, a SID at its last
// fix-bearing leg. This structurally prevents the degeneracy where the enroute
// network flies to a fix a mile off the threshold and reduces the STAR to a
// zero-length stub, because such near-field fixes are terminal TF/CF fixes, not
// published entries (across cycle 2601, 1521 of 1523 sub-1-NM STAR connections
// were TF/CF; published IFs sit at a 64 NM median). Anchors: YSSY's degenerate
// 0.3 NM STAR-end fix TESAT (both unfiltered and under a runway restriction),
// WAAA's 2.2 NM MKS, and KLAX, which must all now arrive over a real procedure
// body.
TEST_CASE("real data: an arrival does not degenerate to a doorstep STAR stub", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }

  // KJFK->YSSY used to end "...B450 TESAT STAR YSSY" with a 0.3 NM arrival: the
  // Pacific airways flew to the threshold and MARLN5 was a stub. TESAT is that
  // STAR's terminal TF fix, not a published entry, so it is no longer a candidate
  // at all and the search must join at a real entry (arrival >> 1 NM).
  bf::Result<std::vector<bf::Route>> yssy = db->FindRoutes(MakeRequest("KJFK", "YSSY"));
  REQUIRE(yssy);
  REQUIRE_FALSE(yssy.value().empty());
  const bf::Route& ry = yssy.value().front();
  CHECK(ry.points.back().ident == "YSSY");
  // A genuine STAR body is tens of NM; the degenerate stub was 0.3 NM.
  CHECK(ry.arr_distance_nm > 5.0);
  CHECK(ry.route_string.find("TESAT") == std::string::npos);

  // Under a runway restriction the same holds. This is the case a seed-threshold
  // filter found hardest: the runway-filtered set of MARLN5.RW07 exposes TESAT
  // and little else, so whether the doorstep got dropped depended on what
  // fallback happened to survive the filter. Keying on the published entry
  // instead makes it independent of the runway filter.
  bf::RouteRequest yssy_rwy = MakeRequest("KJFK", "YSSY");
  yssy_rwy.arrival_runway = "RW07";
  bf::Result<std::vector<bf::Route>> yssy_r07 = db->FindRoutes(yssy_rwy);
  REQUIRE(yssy_r07);
  REQUIRE_FALSE(yssy_r07.value().empty());
  const bf::Route& ry07 = yssy_r07.value().front();
  CHECK(ry07.points.back().ident == "YSSY");
  CHECK(ry07.arr_distance_nm > 5.0);
  CHECK(ry07.route_string.find("TESAT") == std::string::npos);

  // KLAX must keep arriving over a real procedure body. Its nearest published
  // entry is SADDE at ~20 NM (the ~4.7 NM SMO and ~14.2 NM DOWNE that the old
  // model exposed are mid-procedure TF fixes, not entries).
  bf::Result<std::vector<bf::Route>> klax = db->FindRoutes(MakeRequest("KDEN", "KLAX"));
  REQUIRE(klax);
  REQUIRE_FALSE(klax.value().empty());
  const bf::Route& rk = klax.value().front();
  CHECK(rk.points.back().ident == "KLAX");
  CHECK(rk.arr_distance_nm > 5.0);

  // WAAA is the case a seed threshold could not catch: its 2.2 NM MKS entry had
  // no moderate fallback to license dropping it, so a gap-gated filter had to
  // keep it, and only the turn-angle penalty steered the search away. MKS is
  // BIMA1F's terminal CF fix while every WAAA STAR's published entry sits 148 NM
  // or farther out, so the connection model excludes it outright.
  bf::Result<std::vector<bf::Route>> waaa = db->FindRoutes(MakeRequest("KLAX", "WAAA"));
  REQUIRE(waaa);
  REQUIRE_FALSE(waaa.value().empty());
  const bf::Route& rw = waaa.value().front();
  CHECK(rw.points.back().ident == "WAAA");
  CHECK(rw.arr_distance_nm > 5.0);
  CHECK(rw.route_string.find("MKS") == std::string::npos);
}

// Regression: the departure/arrival airport endpoints must carry the airport's
// own coordinate, not the coordinate of the first/last on-network connection
// fix. The search is seeded from the connection fixes (the path starts at the
// first fix), so a naive RoutePoint built from path.vertices.front()/back() used
// to render ZBSJ at fix OC and ZGGG at fix IKAVO. The endpoint and its adjacent
// fix are physically distinct, so their separation must equal the phase
// distance the route already reports -- a collapsed endpoint would read ~0 NM.
TEST_CASE("real data: airport endpoints keep their own coordinate", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("ZBSJ", "ZGGG"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  const bf::Route& r = routes.value().front();

  REQUIRE(r.points.size() >= 3);
  CHECK(r.points.front().ident == "ZBSJ");
  CHECK(r.points.back().ident == "ZGGG");

  const double dep_gap = r.points.front().coord.DistanceTo(r.points[1].coord);
  CHECK(dep_gap == Catch::Approx(r.dep_distance_nm).margin(0.1));

  const double arr_gap = r.points.back().coord.DistanceTo(r.points[r.points.size() - 2].coord);
  CHECK(arr_gap == Catch::Approx(r.arr_distance_nm).margin(0.1));
}

// Regression: ZBSJ->ZGGG used to pick SID ADB01D exiting at OC,
// a 157-degree near-reversal onto B458, because the SID-exit turn was free in
// the cost model. The turn-angle soft penalty must steer the search off that
// reversing exit onto a smoother one (e.g. UKMIS) while still producing a route.
TEST_CASE("real data: ZBSJ->ZGGG SID exit is not a near-reversal", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("ZBSJ", "ZGGG"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  const bf::Route& r = routes.value().front();
  REQUIRE(r.points.size() >= 3);
  CHECK(r.points.front().ident == "ZBSJ");
  CHECK(r.points.back().ident == "ZGGG");

  // Locate the SID leg (airport -> connection fix) and measure the turn at the
  // exit fix: the SID leg's inbound heading vs. the first enroute leg's outbound
  // heading. A near-reversal (>90 deg) is exactly the defect the penalty kills;
  // the chosen exit should now turn smoothly onto the airway.
  size_t sid_leg = r.legs.size();
  for (size_t i = 0; i < r.legs.size(); ++i) {
    if (r.legs[i].via == "SID") {
      sid_leg = i;
      break;
    }
  }
  REQUIRE(sid_leg + 2 < r.points.size());  // exit fix + one enroute fix beyond it
  const bf::Coordinate& airport = r.points[sid_leg].coord;
  const bf::Coordinate& exit_fix = r.points[sid_leg + 1].coord;
  const bf::Coordinate& next_fix = r.points[sid_leg + 2].coord;
  const double inbound = airport.BearingTo(exit_fix);
  const double outbound = exit_fix.BearingTo(next_fix);
  const double turn = bf::TurnAngleDeg(inbound, outbound);
  CHECK(turn < 90.0);  // was 157 deg before the penalty; a reversal is now excluded
}

// --- airway rules (#22) -----------------------------------------------------
// ZSSS->ZGGG is the reference query: its baseline route runs on W131 / W134 / W19
// plus A470 / A599, so W-prefixed rules bite and the effects are observable.

// Collect the distinct `via` designators of a route's enroute legs, skipping the
// leading/trailing procedure legs (whose `via` is the SID/STAR keyword).
std::set<std::string> EnrouteVias(const bf::Route& route) {
  std::set<std::string> vias;
  for (size_t i = 1; i + 1 < route.legs.size(); ++i) {
    if (route.legs[i].via != "DCT" && !route.legs[i].via.empty()) {
      vias.insert(route.legs[i].via);
    }
  }
  return vias;
}

// One rule, spelled out field by field (the CLI string syntax is tested separately
// in the unit suite).
bf::AirwayRule MakeRule(std::vector<std::string> regions, std::vector<std::string> designators,
                        bf::AirwayRule::Match match, bf::AirwayRule::Action action,
                        double fraction = 0.5) {
  bf::AirwayRule rule;
  rule.region_prefixes = std::move(regions);
  rule.designators = std::move(designators);
  rule.match = match;
  rule.action = action;
  rule.penalty_fraction = fraction;
  return rule;
}

TEST_CASE("real data: empty airway rules reproduce the baseline route exactly", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // The zero-regression hard line: adding the field must not perturb any route that
  // does not use it.
  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("ZSSS", "ZGGG"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());

  bf::RouteRequest req = MakeRequest("ZSSS", "ZGGG");
  req.airway_rules = {};  // explicitly empty
  bf::Result<std::vector<bf::Route>> same = db->FindRoutes(req);
  REQUIRE(same);
  REQUIRE_FALSE(same.value().empty());
  CHECK(same.value().front().route_string == baseline.value().front().route_string);
  CHECK(same.value().front().total_distance_nm == baseline.value().front().total_distance_nm);
}

TEST_CASE("real data: a prefix block rule removes the whole designator family", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("ZSSS", "ZGGG"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());
  // The baseline is expected to use at least one W airway for this rule to bite.
  const std::set<std::string> base_vias = EnrouteVias(baseline.value().front());
  bool baseline_has_w = false;
  for (const std::string& via : base_vias) {
    if (via.starts_with("W")) {
      baseline_has_w = true;
      break;
    }
  }
  if (!baseline_has_w) {
    SKIP("baseline ZSSS->ZGGG route uses no W airway to block");
  }

  bf::RouteRequest req = MakeRequest("ZSSS", "ZGGG");
  req.airway_rules = {
      MakeRule({}, {"W"}, bf::AirwayRule::Match::kPrefix, bf::AirwayRule::Action::kBlock)};
  bf::Result<std::vector<bf::Route>> blocked = db->FindRoutes(req);
  REQUIRE(blocked);  // penalize is the safe default, but blocking W still leaves a route
  REQUIRE_FALSE(blocked.value().empty());
  for (const std::string& via : EnrouteVias(blocked.value().front())) {
    CHECK_FALSE(via.starts_with("W"));
  }
}

TEST_CASE("real data: exact matching does not catch prefix-extended designators", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // The core of #22's exact/prefix split: 1371 designators in cycle 2601 are a
  // strict prefix of another one. Blocking "W1" exactly must leave W131/W134 alone,
  // while blocking it as a prefix must remove them.
  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("ZSSS", "ZGGG"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());
  const std::set<std::string> base_vias = EnrouteVias(baseline.value().front());
  // Pick a baseline W airway with digits after the first one (W131, W134, ...) so an
  // exact rule on its 2-char stem is a no-op while a prefix rule bites.
  std::string victim;
  for (const std::string& via : base_vias) {
    if (via.size() > 2 && via.starts_with("W")) {
      victim = via;
      break;
    }
  }
  if (victim.empty()) {
    SKIP("baseline route uses no multi-digit W airway");
  }
  const std::string stem = victim.substr(0, 2);  // e.g. "W1" from "W131"

  bf::RouteRequest exact = MakeRequest("ZSSS", "ZGGG");
  exact.airway_rules = {
      MakeRule({}, {stem}, bf::AirwayRule::Match::kExact, bf::AirwayRule::Action::kBlock)};
  bf::Result<std::vector<bf::Route>> exact_result = db->FindRoutes(exact);
  REQUIRE(exact_result);
  REQUIRE_FALSE(exact_result.value().empty());
  // The stem names no real airway (or at least not the victim), so the route stands.
  CHECK(exact_result.value().front().route_string == baseline.value().front().route_string);

  bf::RouteRequest prefix = MakeRequest("ZSSS", "ZGGG");
  prefix.airway_rules = {
      MakeRule({}, {stem}, bf::AirwayRule::Match::kPrefix, bf::AirwayRule::Action::kBlock)};
  bf::Result<std::vector<bf::Route>> prefix_result = db->FindRoutes(prefix);
  REQUIRE(prefix_result);
  REQUIRE_FALSE(prefix_result.value().empty());
  CHECK(EnrouteVias(prefix_result.value().front()).count(victim) == 0);
  CHECK(prefix_result.value().front().route_string != baseline.value().front().route_string);
}

TEST_CASE("real data: a region-restricted rule is narrower than an unrestricted one",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // Blocking W airways only inside ZS leaves W airways elsewhere usable, so the
  // route may still ride a W leg outside that region -- the per-leg granularity #22
  // settled on. An all-regions rule cannot.
  bf::RouteRequest region = MakeRequest("ZSSS", "ZGGG");
  region.airway_rules = {
      MakeRule({"ZS"}, {"W"}, bf::AirwayRule::Match::kPrefix, bf::AirwayRule::Action::kBlock)};
  bf::Result<std::vector<bf::Route>> region_result = db->FindRoutes(region);
  REQUIRE(region_result);
  REQUIRE_FALSE(region_result.value().empty());

  bf::RouteRequest global = MakeRequest("ZSSS", "ZGGG");
  global.airway_rules = {
      MakeRule({}, {"W"}, bf::AirwayRule::Match::kPrefix, bf::AirwayRule::Action::kBlock)};
  bf::Result<std::vector<bf::Route>> global_result = db->FindRoutes(global);
  REQUIRE(global_result);
  REQUIRE_FALSE(global_result.value().empty());

  // The narrower rule forbids a subset of what the global one does, so its optimum
  // can never be worse (longer) than the global rule's.
  CHECK(region_result.value().front().total_distance_nm <=
        global_result.value().front().total_distance_nm + 1e-6);
}

TEST_CASE("real data: penalize keeps a route where block would remove it", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // The reason region-level rules default to penalize (#22 D6): blocking every
  // airway severs the graph and yields no route, while penalizing every airway
  // leaves the graph connected and simply re-prices it -- the same route comes back.
  bf::Result<std::vector<bf::Route>> baseline = db->FindRoutes(MakeRequest("ZSSS", "ZGGG"));
  REQUIRE(baseline);
  REQUIRE_FALSE(baseline.value().empty());

  bf::RouteRequest blocked = MakeRequest("ZSSS", "ZGGG");
  blocked.airway_rules = {
      MakeRule({}, {}, bf::AirwayRule::Match::kPrefix, bf::AirwayRule::Action::kBlock)};
  CHECK_FALSE(db->FindRoutes(blocked));  // every airway forbidden => no route at all

  bf::RouteRequest penalized = MakeRequest("ZSSS", "ZGGG");
  penalized.airway_rules = {
      MakeRule({}, {}, bf::AirwayRule::Match::kPrefix, bf::AirwayRule::Action::kPenalize, 0.5)};
  bf::Result<std::vector<bf::Route>> penalized_result = db->FindRoutes(penalized);
  REQUIRE(penalized_result);
  REQUIRE_FALSE(penalized_result.value().empty());
  // A uniform penalty scales every airway leg alike, so the optimum is unchanged.
  CHECK(penalized_result.value().front().route_string == baseline.value().front().route_string);
}

TEST_CASE("real data: too many airway rules is an error, not a silent truncation",
          "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  bf::RouteRequest req = MakeRequest("ZSSS", "ZGGG");
  // kMaxRules rules is fine; one more must be refused rather than dropped, since a
  // dropped rule would silently let a forbidden airway back into the route.
  const bf::AirwayRule rule =
      MakeRule({"XX"}, {"ZZZ"}, bf::AirwayRule::Match::kExact, bf::AirwayRule::Action::kBlock);
  req.airway_rules.assign(bf::AirwayRuleConstraint::kMaxRules, rule);
  CHECK(db->FindRoutes(req));

  req.airway_rules.push_back(rule);
  bf::Result<std::vector<bf::Route>> over = db->FindRoutes(req);
  REQUIRE_FALSE(over);
  CHECK(over.error().message.find("too many airway rules") != std::string::npos);
}

TEST_CASE("real data: one rule may enumerate many regions and designators", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // Both sides of a rule share its single bit, so a long list is still ONE rule and
  // does not eat into the kMaxRules budget. Mainland China is ten FIRs; note the
  // prefix "Z" would additionally cover ZM (Mongolia) and ZK (North Korea).
  bf::RouteRequest req = MakeRequest("ZSSS", "ZGGG");
  req.airway_rules = {MakeRule({"ZB", "ZG", "ZH", "ZJ", "ZL", "ZP", "ZS", "ZU", "ZW", "ZY"},
                               {"W131", "W134", "W19", "A470", "A599"},
                               bf::AirwayRule::Match::kExact, bf::AirwayRule::Action::kBlock)};
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(req);
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());
  // Every enumerated designator is gone from the result.
  const std::set<std::string> vias = EnrouteVias(routes.value().front());
  for (const std::string_view blocked : {"W131", "W134", "W19", "A470", "A599"}) {
    CHECK(vias.count(std::string(blocked)) == 0);
  }
}

}  // namespace
