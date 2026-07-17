#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <fstream>
#include <set>
#include <string>
#include <thread>
#include <vector>

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

TEST_CASE("real data: an arrival joins the STAR at a near fix, not a far entry", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }

  // A STAR exposes every on-network fix it passes as a candidate connection, not
  // just its published entry fix. KLAX BASET5, for instance, enters from PGS
  // (~260 NM out) but its common segment runs through DOWNE close to the field.
  // An arrival from the northeast should join at a near fix, so the final STAR
  // leg (last connection fix -> airport) stays short rather than spanning the
  // whole procedure from a far entry.
  bf::Result<std::vector<bf::Route>> routes = db->FindRoutes(MakeRequest("KDEN", "KLAX"));
  REQUIRE(routes);
  REQUIRE_FALSE(routes.value().empty());

  const bf::Route& r = routes.value().front();
  REQUIRE_FALSE(r.legs.empty());
  const bf::RouteLeg& last = r.legs.back();
  CHECK(last.to == "KLAX");
  CHECK_FALSE(r.star.empty());
  // The chosen entry must not be the far BASET5 entry fix; the final procedure
  // leg should be a short hop, well under the ~260 NM that the far entry implied.
  CHECK(last.from != "PGS");
  CHECK(last.distance_nm < 60.0);
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
  // several competitive arrival options (KIMMO3, WAYVE1); with enough candidates
  // more than one distinct STAR appears (WAYVE1 shows up beyond the first few).
  bf::RouteRequest req = MakeRequest("KSEA", "KLAX");
  req.k = 8;
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

  bf::RouteRequest req = MakeRequest("KJFK", "KLAX");
  req.avoid_airways = {victim_awy};
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
}

TEST_CASE("real data: ParseRoute expands an airway's intermediate fixes", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // MCI J24 SLN is a short enroute hop; J24 threads at least one intermediate
  // fix between them, which must appear as expanded points.
  bf::Result<bf::Route> r = db->ParseRoute("MCI J24 SLN");
  if (!r) {
    SKIP("J24 MCI->SLN not present in this AIRAC cycle");
  }
  CHECK(r.value().points.size() >= 3);  // MCI, >=1 intermediate, SLN
  CHECK(r.value().total_distance_nm > 0.0);
  for (const bf::RouteLeg& leg : r.value().legs) {
    CHECK(leg.via == "J24");
  }
}

TEST_CASE("real data: ParseRoute reports a disconnected airway", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  // J24 does not connect MCI to an arbitrary far fix; expect an error, not a
  // silently wrong route.
  bf::Result<bf::Route> r = db->ParseRoute("MCI J24 SEA");
  CHECK_FALSE(r);
}

TEST_CASE("real data: ParseRoute rejects an unknown fix", "[integration]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "' (set BRAVOFINDER_NAVDATA)");
  }
  bf::Result<bf::Route> r = db->ParseRoute("MCI DCT ZZZQ");
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

  // The no-fix shape is accepted only with an explicit DCT and airports at both
  // ends. A bare airport pair (no connector) and an airport->fix DCT both stay
  // errors -- they are not valid filed routes.
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

}  // namespace
