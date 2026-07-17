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

TEST_CASE("SmallVec: copy assignment replaces existing contents", "[small_vec]") {
  SmallVec<int, 4> a;
  for (int i = 0; i < 6; ++i) {  // heap-backed source
    a.push_back(i);
  }
  SmallVec<int, 4> b;
  b.push_back(-1);  // b starts non-empty and inline
  b = a;
  REQUIRE(b.size() == 6);
  for (int i = 0; i < 6; ++i) {
    CHECK(b[i] == i);
  }
  // Independent storage: mutating a leaves b untouched.
  a.push_back(42);
  CHECK(b.size() == 6);
}

TEST_CASE("SmallVec: move assignment transfers and empties the source", "[small_vec]") {
  SmallVec<int, 4> a;
  for (int i = 0; i < 7; ++i) {  // heap-backed source
    a.push_back(i + 1);
  }
  SmallVec<int, 4> b;
  b.push_back(-1);
  b = std::move(a);
  REQUIRE(b.size() == 7);
  for (int i = 0; i < 7; ++i) {
    CHECK(b[i] == i + 1);
  }
  CHECK(a.empty());  // moved-from is left empty
}

TEST_CASE("SmallVec: self copy- and move-assignment are safe", "[small_vec]") {
  SmallVec<int, 4> a;
  for (int i = 0; i < 6; ++i) {
    a.push_back(i);
  }
  SmallVec<int, 4>& ref = a;
  a = ref;  // self copy-assign: must not corrupt or free-then-read
  REQUIRE(a.size() == 6);
  CHECK(a.back() == 5);
  a = std::move(ref);  // self move-assign: guarded by this != &other, a no-op
  REQUIRE(a.size() == 6);
  CHECK(a.front() == 0);
}

TEST_CASE("SmallVec: push_back after a move keeps the small-buffer optimization", "[small_vec]") {
  // A moved-from vector must be left as a valid empty inline vector: a later
  // push_back should refill the inline buffer, not immediately heap-allocate
  // from a zeroed capacity.
  SmallVec<int, 4> a;
  a.push_back(1);
  a.push_back(2);
  SmallVec<int, 4> b = std::move(a);
  REQUIRE(a.empty());
  // Reuse the moved-from vector.
  for (int i = 0; i < 4; ++i) {
    a.push_back(i * 2);
  }
  REQUIRE(a.size() == 4);
  CHECK(a[0] == 0);
  CHECK(a[3] == 6);
  // b is unaffected by reusing a.
  REQUIRE(b.size() == 2);
  CHECK(b[0] == 1);
  CHECK(b[1] == 2);
}
