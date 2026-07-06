#include "core/result.h"

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>

namespace {

using bf::Error;
using bf::ErrorCode;
using bf::Result;

TEST_CASE("Ok holds a value", "[result]") {
  Result<int> r = Result<int>::Ok(42);
  REQUIRE(r.has_value());
  CHECK(static_cast<bool>(r));
  CHECK(r.value() == 42);
}

TEST_CASE("Err holds an error", "[result]") {
  Result<int> r = Result<int>::Err(Error(ErrorCode::kNoRoute, "no route found"));
  REQUIRE_FALSE(r.has_value());
  CHECK_FALSE(static_cast<bool>(r));
  CHECK(r.error().code == ErrorCode::kNoRoute);
  CHECK(r.error().message == "no route found");
}

TEST_CASE("value_or returns fallback on error", "[result]") {
  Result<int> ok = Result<int>::Ok(7);
  Result<int> err = Result<int>::Err(Error(ErrorCode::kUnknown, ""));
  CHECK(ok.value_or(-1) == 7);
  CHECK(err.value_or(-1) == -1);
}

TEST_CASE("works with a move-only value type", "[result]") {
  Result<std::string> r = Result<std::string>::Ok("KJFK");
  REQUIRE(r.has_value());
  CHECK(std::move(r).value() == "KJFK");
}

TEST_CASE("value_or on an rvalue moves the value out", "[result]") {
  // The && overload lets a move-only value be extracted via value_or on a
  // temporary Result without copying. A unique_ptr is the canonical move-only
  // type: if only the const& overload existed, this would fail to compile.
  auto make = []() -> Result<std::unique_ptr<int>> {
    return Result<std::unique_ptr<int>>::Ok(std::make_unique<int>(99));
  };
  std::unique_ptr<int> p = std::move(make()).value_or(nullptr);
  REQUIRE(p != nullptr);
  CHECK(*p == 99);

  // Fallback is taken on an error rvalue.
  auto fail = []() -> Result<std::unique_ptr<int>> {
    return Result<std::unique_ptr<int>>::Err(Error(ErrorCode::kUnknown, ""));
  };
  std::unique_ptr<int> q = std::move(fail()).value_or(nullptr);
  CHECK(q == nullptr);
}

}  // namespace
