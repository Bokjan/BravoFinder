#include "io/loaders/loader_registry.h"

#include <catch2/catch_test_macros.hpp>

#include "core/result.h"

TEST_CASE("loader registry: xplane12 resolves to a loader", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("xplane12");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "xplane12");
}

TEST_CASE("loader registry: an unknown name is a clean error", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("littlenavmap");
  REQUIRE_FALSE(loader);
  CHECK(loader.error().code == bf::ErrorCode::kInvalidArgument);
}
