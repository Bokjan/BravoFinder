// SPDX-License-Identifier: MIT
// http_handlers_test.cc — the "router mapping layer": each shared bf::service
// handler, given parsed args + a real NavDatabase, returns the right
// HTTP-style status. This is the fast, socket-free coverage of the 200 / 400 /
// 404 / 422 contract the HTTP transport relies on (see docs/http-service). Uses
// real navigation data; SKIPs when it is absent.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "handlers.h"
#include "io/nav_database.h"
#include "queries.h"
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

TEST_CASE("http handlers: elapsed_ms is set on success, zero on validation error",
          "[integration][http]") {
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
      Run("parse_route", R"({"route":"KJFK DEEZZ5 CANDR DCT KLAX"})", *db);
  REQUIRE(parsed.status == 200);
  CHECK(parsed.elapsed_ms < 10000);
  const bf::service::HandlerResult lookup = Run("lookup_airports", R"({"ids":["KJFK"]})", *db);
  REQUIRE(lookup.status == 200);
  CHECK(lookup.elapsed_ms < 10000);
  // 400 validation paths leave elapsed_ms at 0 (rejected before meaningful work).
  CHECK(Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":0})", *db).elapsed_ms == 0);
  CHECK(Run("find_routes", R"({"departure":"KJFK"})", *db).elapsed_ms == 0);
  CHECK(Run("lookup_airports", R"({})", *db).elapsed_ms == 0);
  // 422 / 404 paths timed an engine call: elapsed_ms is set (may be 0 if sub-ms)
  // and must stay under the same stuck-query ceiling.
  CHECK(Run("find_routes", R"({"departure":"ZZ_NOPE_ZZ","arrival":"KLAX"})", *db).elapsed_ms <
        10000);
  CHECK(Run("parse_route", R"({"route":"KJFK ZZ_NOPE9 KLAX"})", *db).elapsed_ms < 10000);
  CHECK(Run("lookup_airports", R"({"ids":["ZZ_NOPE_ZZ"]})", *db).elapsed_ms < 10000);
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
  CHECK(Run("parse_route", R"({"route":"KJFK DEEZZ5 CANDR DCT KLAX"})", *db).status == 200);
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

TEST_CASE("http handlers: airway_rules normalize lowercase regions and designators",
          "[integration][http]") {
  // Regression guard for the case-sensitivity gap: the navigation data stores
  // region codes and designators upper-case, so a lowercase user input must be
  // normalized before matching (see ParseRuleStringList's ToUpper) -- otherwise
  // a lowercase "w" silently matches nothing and the route keeps its W legs.
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  // ZSSS->ZGGG is the reference query whose baseline runs on W131 / W134 / W19.
  // Use region_prefixes:["*"] (any region; empty [] is rejected) and a
  // lowercase designator "w" with match:"prefix": the rule must block the whole
  // W family anywhere. (A region-limited rule would legitimately keep W legs
  // outside that region -- e.g. W45 near ZGGG -- so an unrestricted rule is the
  // clean assertion that normalization happened.)
  const bf::service::HandlerResult blocked = Run(
      "find_routes",
      R"({"departure":"ZSSS","arrival":"ZGGG","k":1,"airway_rules":[{"region_prefixes":["*"],"designators":["w"],"match":"prefix","action":"block"}]})",
      *db);
  REQUIRE(blocked.status == 200);
  rapidjson::Document doc;
  doc.Parse(blocked.body.c_str());
  REQUIRE_FALSE(doc.HasParseError());
  REQUIRE(doc.IsArray());
  REQUIRE(doc.Size() > 0);
  // Walk the enroute legs (skip the leading SID and trailing STAR legs, whose
  // `via` is the procedure keyword) and require no "W" designator survives.
  const rapidjson::Value& legs = doc[0]["legs"];
  bool has_w = false;
  for (rapidjson::SizeType i = 1; i + 1 < legs.Size(); ++i) {
    const std::string via = legs[i]["via"].GetString();
    if (via != "DCT" && !via.empty() && via[0] == 'W') {
      has_w = true;
      break;
    }
  }
  CHECK_FALSE(has_w);
  // Sanity: the lowercase rule genuinely changed the route (the W legs are gone),
  // proving the lowercase "w" input was normalized rather than silently ignored.
  const bf::service::HandlerResult baseline =
      Run("find_routes", R"({"departure":"ZSSS","arrival":"ZGGG","k":1})", *db);
  REQUIRE(baseline.status == 200);
  const std::string base_string = baseline.body;
  CHECK(blocked.body != base_string);
}

TEST_CASE("http handlers: empty ids and incomplete airway_rules are 400", "[integration][http]") {
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  CHECK(Run("lookup_airports", R"({"ids":[]})", *db).status == 400);
  // Both region_prefixes and designators are required; an empty rule object is
  // rejected (use explicit ["*"] for "any", not omission or []).
  CHECK(
      Run("find_routes", R"({"departure":"KJFK","arrival":"KLAX","k":1,"airway_rules":[{}]})", *db)
          .status == 400);
}

TEST_CASE("typed lookups reject an id list over kMaxIdListSize", "[integration][http]") {
  // The HTTP adapter (ParseIdList) caps ids at kMaxIdListSize, but the CLI calls
  // the typed entries directly. The cap must live in the typed entry too so the
  // CLI path is bounded the same as HTTP — exercise it without going through the
  // adapter by calling bf::service::Lookup* with an oversized vector.
  const bf::NavDatabase* db = SharedDb();
  if (db == nullptr) {
    SKIP("navigation data not found in '" << NavDataDir() << "'");
  }
  const std::vector<std::string> too_many(bf::service::kMaxIdListSize + 1, "KJFK");
  CHECK(bf::service::LookupAirports(*db, too_many, bf::service::OutputFormat::kJson).status == 400);
  CHECK(bf::service::LookupWaypoints(*db, too_many, bf::service::OutputFormat::kJson).status ==
        400);
  CHECK(bf::service::LookupAirways(*db, too_many, bf::service::OutputFormat::kJson).status == 400);
  CHECK(bf::service::LookupNavaidDetails(*db, too_many, bf::service::OutputFormat::kJson).status ==
        400);
  CHECK(bf::service::LookupHolds(*db, too_many, bf::service::OutputFormat::kJson).status == 400);
}
