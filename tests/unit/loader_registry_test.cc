// SPDX-License-Identifier: MIT
#include "io/loaders/loader_registry.h"

#include <catch2/catch_test_macros.hpp>

#include "core/result.h"

TEST_CASE("loader registry: xplane12 resolves to a loader", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("xplane12");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "xplane12");
}

TEST_CASE("loader registry: dfd1 resolves to a loader", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("dfd1");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "dfd1");
}

TEST_CASE("loader registry: dfd2 resolves to a loader", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("dfd2");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "dfd2");
}

TEST_CASE("loader registry: fenix resolves to a loader", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("fenix");
  REQUIRE(loader);
  REQUIRE(loader.value() != nullptr);
  CHECK(loader.value()->name() == "fenix");
}

TEST_CASE("loader registry: capabilities match each loader's fidelity", "[unit][loader]") {
  {
    bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("xplane12");
    REQUIRE(loader);
    const bf::LoaderCapabilities caps = loader.value()->capabilities();
    CHECK(caps.airway_direction);
    CHECK(caps.altitude_bands);
    CHECK(caps.mora_grid);
    CHECK(caps.msa_sectors);
  }
  {
    bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("fenix");
    REQUIRE(loader);
    const bf::LoaderCapabilities caps = loader.value()->capabilities();
    CHECK_FALSE(caps.airway_direction);
    CHECK_FALSE(caps.altitude_bands);
    CHECK(caps.mora_grid);
    CHECK_FALSE(caps.msa_sectors);
  }
}

TEST_CASE("loader registry: an unknown name is a clean error", "[unit][loader]") {
  bf::Result<std::unique_ptr<bf::Loader>> loader = bf::MakeLoader("littlenavmap");
  REQUIRE_FALSE(loader);
  CHECK(loader.error().code == bf::ErrorCode::kInvalidArgument);
}
