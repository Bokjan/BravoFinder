// http_handlers_test.cc — the "router mapping layer": each shared bf::service
// handler, given parsed args + a real NavDatabase, returns the right
// HTTP-style status. This is the fast, socket-free coverage of the 200 / 400 /
// 404 / 422 contract the HTTP transport relies on (design section 四). Uses real
// navigation data; SKIPs when it is absent.

#include <catch2/catch_test_macros.hpp>
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
  // A well-formed request with an unknown endpoint is a semantic failure (422).
  CHECK(Run("find_routes", R"({"departure":"ZZ_NOPE_ZZ","arrival":"KLAX"})", *db).status == 422);
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
