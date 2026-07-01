#include "core/util/small_vec.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <vector>

using bf::SmallVec;

TEST_CASE("SmallVec: empty and inline-only", "[small_vec]") {
  SmallVec<int, 4> v;
  CHECK(v.empty());
  CHECK(v.size() == 0);
  CHECK(v.begin() == v.end());
}

TEST_CASE("SmallVec: fills inline capacity, no heap", "[small_vec]") {
  SmallVec<int, 4> v;
  for (int i = 0; i < 4; ++i) {
    v.push_back(i * 10);
  }
  REQUIRE(v.size() == 4);
  CHECK(v[0] == 0);
  CHECK(v[3] == 30);
  CHECK(v.front() == 0);
  CHECK(v.back() == 30);

  // Range iteration yields the elements in order.
  std::vector<int> seen(v.begin(), v.end());
  CHECK(seen == std::vector<int>{0, 10, 20, 30});
}

TEST_CASE("SmallVec: overflows to heap past N", "[small_vec]") {
  SmallVec<int, 4> v;
  for (int i = 0; i < 8; ++i) {
    v.push_back(i);
  }
  REQUIRE(v.size() == 8);
  for (int i = 0; i < 8; ++i) {
    CHECK(v[i] == i);
  }
  // The long tail is intact after the inline->heap transition and a re-grow.
  CHECK(v.back() == 7);
}

TEST_CASE("SmallVec: move preserves contents, source empties", "[small_vec]") {
  SmallVec<int, 4> a;
  for (int i = 0; i < 6; ++i) {
    a.push_back(i + 1);  // forces heap so we exercise the heap move path
  }
  SmallVec<int, 4> b = std::move(a);
  REQUIRE(b.size() == 6);
  for (int i = 0; i < 6; ++i) {
    CHECK(b[i] == i + 1);
  }
  CHECK(a.empty());  // moved-from is left empty and inline (owns nothing)
}

TEST_CASE("SmallVec: copy duplicates contents independently", "[small_vec]") {
  SmallVec<int, 4> a;
  for (int i = 0; i < 5; ++i) {
    a.push_back(i);
  }
  SmallVec<int, 4> b = a;
  REQUIRE(b.size() == 5);
  // Mutating the copy must not affect the original.
  b.push_back(99);
  CHECK(a.size() == 5);
  CHECK(b.size() == 6);
  CHECK(b.back() == 99);
}
