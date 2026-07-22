// http_handlers_test.cc — the "router mapping layer": each shared bf::service
// handler, given parsed args + a real NavDatabase, returns the right
// HTTP-style status. This is the fast, socket-free coverage of the 200 / 400 /
// 404 / 422 contract the HTTP transport relies on (see docs/http-service). Uses
// real navigation data; SKIPs when it is absent.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>
#include <unordered_map>

#include "handlers.h"
#include "io/nav_database.h"
#include "rapidjson/document.h"
#include "test_bfdb.h"

namespace {

using bf::test::NavDataDir;

const bf::NavDatabase* SharedDb() {
  static bf::Result<bf::NavDatabase> db = bf::test::OpenReadOnlyDb();
  return db ? &db.value() : nullptr;
}

// The handlers indexed by name, built once.
const std::unordered_map<std::string, bf::service::QueryHandler>& Handlers() {
  static const std::unordered_map<std::string, bf::service::QueryHandler> map = [] {
    std::unordered_map<std::string, bf::service::QueryHandler> m;
    for (bf::service::NamedHandler& nh : bf::service::MakeHandlers()) {
      m.emplace(std::move(nh.name), std::move(nh.handler));
    }
    return m;
  }();
  return map;
}

// Run the named handler with a JSON argument string and return the result.
bf::service::HandlerResult Run(const std::string& name, const char* json,
                               const bf::NavDatabase& db) {
  rapidjson::Document args;
  args.Parse(json);
  REQUIRE_FALSE(args.HasParseError());
  return Handlers().at(name)(args, db);
}

}  // namespace

TEST_CASE("http handlers: find_routes status mapping", "[integration][http]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":1})", *db).status == 200);
  // Missing a required field is a bad request.
  CHECK(Run("find_routes", R"({"departure":"KJFK"})", *db).status == 400);
  // k < 1 is rejected before FindRoutes.
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":0})", *db).status == 400);
  // k == 15 is the upper bound (accepted); k == 16 exceeds the cap (rejected).
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":15})", *db).status == 200);
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":16})", *db).status == 400);
  // Altitude bounds must be ordered and non-negative.
  CHECK(
      Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","min_fl":400,"max_fl":300})", *db)
          .status == 400);
  CHECK(
      Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","min_fl":-100,"max_fl":300})", *db)
          .status == 400);
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","max_fl":-50})", *db).status ==
        400);
  // A well-formed request with an unknown endpoint is a semantic failure (422).
  CHECK(Run("find_routes", R"({"departure":"ZZ_NOPE_ZZ","arrival":"KLAX"})", *db).status == 422);
}

TEST_CASE("http handlers: elapsed_ms is set on success, zero on error", "[integration][http]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // A successful database call reports a plausible compute cost, well under a
  // 10 s ceiling that would signal a stuck query. find_routes is reliably heavy
  // (a full A*/Yen search), so it also clears > 0; parse_route and the lookups
  // can legitimately finish in under a millisecond and round to 0.
  const bf::service::HandlerResult routes =
      Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":1})", *db);
  REQUIRE(routes.status == 200);
  CHECK(routes.elapsed_ms > 0);
  CHECK(routes.elapsed_ms < 10000);
  const bf::service::HandlerResult parsed =
      Run("parse_route", R"({"route":"KJFK DEEZZ5 CANDR"})", *db);
  REQUIRE(parsed.status == 200);
  CHECK(parsed.elapsed_ms < 10000);
  const bf::service::HandlerResult lookup = Run("lookup_airports", R"({"ids":["KJFK"]})", *db);
  REQUIRE(lookup.status == 200);
  CHECK(lookup.elapsed_ms < 10000);
  // Every error path leaves elapsed_ms at 0 (no meaningful compute happened).
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":0})", *db).elapsed_ms == 0);
  CHECK(Run("find_routes", R"({"departure":"KJFK"})", *db).elapsed_ms == 0);
  CHECK(Run("find_routes", R"({"departure":"ZZ_NOPE_ZZ","arrival":"KLAX"})", *db).elapsed_ms == 0);
  CHECK(Run("parse_route", R"({"route":"KJFK ZZ_NOPE9 KLAX"})", *db).elapsed_ms == 0);
  CHECK(Run("lookup_airports", R"({"ids":["ZZ_NOPE_ZZ"]})", *db).elapsed_ms == 0);
  CHECK(Run("lookup_airports", R"({})", *db).elapsed_ms == 0);
}

TEST_CASE("http handlers: find_routes emits full-precision coordinates", "[integration][http]") {
  // Regression guard: the route writer must run at 6 dp, not 2 -- 2 dp truncates
  // point lat/lon to ~1.1 km. A real route's points are not all 2-dp values, so
  // at least one coordinate must differ from its 2-dp rounding by > 1e-4.
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const bf::service::HandlerResult result =
      Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":1})", *db);
  REQUIRE(result.status == 200);
  rapidjson::Document doc;
  doc.Parse(result.body.c_str());
  REQUIRE_FALSE(doc.HasParseError());
  REQUIRE(doc.IsArray());
  REQUIRE(doc.Size() >= 1);
  const rapidjson::Value& points = doc[0]["points"];
  REQUIRE(points.IsArray());
  REQUIRE(points.Size() > 0);
  bool found_full_precision = false;
  for (const rapidjson::Value& p : points.GetArray()) {
    const double lat = p["lat"].GetDouble();
    const double rounded = std::round(lat * 100.0) / 100.0;
    if (std::abs(lat - rounded) > 1e-4) {
      found_full_precision = true;
      break;
    }
  }
  CHECK(found_full_precision);
}

TEST_CASE("http handlers: parse_route status mapping", "[integration][http]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  CHECK(Run("parse_route", R"({"route":"KJFK DEEZZ5 CANDR"})", *db).status == 200);
  CHECK(Run("parse_route", R"({})", *db).status == 400);
  // A bad token is a semantic failure (422), not a bad request.
  CHECK(Run("parse_route", R"({"route":"KJFK ZZ_NOPE9 KLAX"})", *db).status == 422);
}

TEST_CASE("http handlers: batch lookup status mapping", "[integration][http]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // A partial hit is 200; every id missing is 404; a missing ids array is 400.
  CHECK(Run("lookup_airports", R"({"ids":["KJFK","ZZ_NOPE_ZZ"]})", *db).status == 200);
  CHECK(Run("lookup_airports", R"({"ids":["ZZ_NOPE_ZZ"]})", *db).status == 404);
  CHECK(Run("lookup_airports", R"({})", *db).status == 400);
  // Grouped lookups follow the same rule (empty group == miss).
  CHECK(Run("lookup_waypoints", R"({"ids":["ZZ_NOPE_ZZ"]})", *db).status == 404);
}

TEST_CASE("http handlers: procedure-legs status mapping", "[integration][http]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  CHECK(Run("lookup_procedure_legs", R"({"airport":"KJFK"})", *db).status == 400);
  // KIKR is a real airport with no CIFP procedures: an unknown procedure is 404.
  CHECK(Run("lookup_procedure_legs", R"({"airport":"KIKR","procedure":"NOPE"})", *db).status ==
        404);
}
