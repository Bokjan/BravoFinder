// SPDX-License-Identifier: MIT
// Microbenchmark for issue #3: does replacing GraphBuilder's three lookup maps
// (unordered_map) with a sorted vector + binary search regress lookup latency?
//
// This is a STANDALONE measurement harness, deliberately NOT wired into
// GraphBuilder. It loads a real .bfdb, extracts the real per-vertex idents
// (~270k), builds both index forms side by side, and times:
//   1. index build (unordered_map insert vs sort)
//   2. lookup latency over a real query workload
//   3. memory (sizeof-based estimate)
// so we can decide with data whether the ~20 MB saving is worth any lookup cost
// before touching production code.
//
// Not part of the default build; enable with -DBRAVOFINDER_BUILD_BENCH=ON.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/domain/fixed_string.h"
#include "core/domain/ident.h"
#include "io/cache/unified_cache.h"

namespace {

using Clock = std::chrono::steady_clock;

double MsSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// An 8-byte, region-less key candidate for airport_index_ / ident_all_ (see #3):
// length-prefixed, no NUL, ordered by length + memcmp.
struct FixedName8 {
  static constexpr int kCap = 7;
  uint8_t len = 0;
  char text[kCap] = {};
  static FixedName8 From(std::string_view s) {
    FixedName8 f;
    f.len = static_cast<uint8_t>(s.size() < kCap ? s.size() : kCap);
    std::memcpy(f.text, s.data(), f.len);
    return f;
  }
  int cmp(const FixedName8& o) const {
    int c = std::memcmp(text, o.text, std::min(len, o.len));
    return c != 0 ? c : (int(len) - int(o.len));
  }
  bool operator<(const FixedName8& o) const { return cmp(o) < 0; }
};
static_assert(sizeof(FixedName8) == 8, "FixedName8 must be 8 bytes");

}  // namespace

int main(int argc, char** argv) {
  const std::string db = argc > 1 ? argv[1] : "navdata/bfdb/nav.bfdb";
  const int rounds = argc > 2 ? std::atoi(argv[2]) : 20;

  bf::Result<bf::UnifiedData> data = bf::UnifiedCache::Open(db);
  if (!data) {
    std::fprintf(stderr, "cannot open %s: %s\n", db.c_str(), data.error().message.c_str());
    std::fprintf(stderr, "usage: %s [nav.bfdb] [rounds]\n", argv[0]);
    return 1;
  }
  const std::vector<bf::FixedIdent>& idents = data.value().graph.idents;
  const size_t V = idents.size();
  std::printf("db=%s  V=%zu vertices  rounds=%d\n\n", db.c_str(), V, rounds);

  // Materialize the real (ident, region) list once, so index-build timing does
  // not include the FixedIdent->string cost.
  std::vector<std::pair<std::string, std::string>> keys;
  keys.reserve(V);
  for (const bf::FixedIdent& fi : idents) {
    keys.emplace_back(std::string(fi.IdentView()), std::string(fi.RegionView()));
  }

  // ---- 1. Build: unordered_map<Ident,int> vs sorted vector<(FixedIdent,int)> ----
  double t_map_build = 0.0;
  double t_vec_build = 0.0;
  std::unordered_map<bf::Ident, int> map_index;
  std::vector<std::pair<bf::FixedIdent, int>> vec_index;
  {
    const auto t0 = Clock::now();
    map_index.reserve(V);
    for (size_t i = 0; i < V; ++i) {
      map_index.emplace(bf::Ident(keys[i].first, keys[i].second), static_cast<int>(i));
    }
    t_map_build = MsSince(t0);
  }
  {
    const auto t0 = Clock::now();
    vec_index.reserve(V);
    for (size_t i = 0; i < V; ++i) {
      vec_index.emplace_back(bf::FixedIdent::FromParts(keys[i].first, keys[i].second),
                             static_cast<int>(i));
    }
    std::sort(vec_index.begin(), vec_index.end(), [](const auto& a, const auto& b) {
      if (int c = a.first.IdentView().compare(b.first.IdentView())) {
        return c < 0;
      }
      return a.first.RegionView() < b.first.RegionView();
    });
    t_vec_build = MsSince(t0);
  }

  // ---- 2. Lookup workload: every real key, shuffled, plus 10% misses ----
  std::mt19937 rng(12345);
  std::vector<size_t> order(V);
  for (size_t i = 0; i < V; ++i) {
    order[i] = i;
  }
  std::shuffle(order.begin(), order.end(), rng);

  auto vec_find = [&](const bf::Ident& q) -> int {
    const bf::FixedIdent fq = bf::FixedIdent::FromParts(q.ident, q.region);
    auto it = std::lower_bound(vec_index.begin(), vec_index.end(), fq,
                               [](const auto& a, const bf::FixedIdent& b) {
                                 if (int c = a.first.IdentView().compare(b.IdentView())) {
                                   return c < 0;
                                 }
                                 return a.first.RegionView() < b.RegionView();
                               });
    if (it != vec_index.end() && it->first == fq) {
      return it->second;
    }
    return -1;
  };

  int64_t sink = 0;
  double t_map_find = 0.0;
  double t_vec_find = 0.0;
  {
    const auto t0 = Clock::now();
    for (int rep = 0; rep < rounds; ++rep) {
      for (size_t idx : order) {
        auto it = map_index.find(bf::Ident(keys[idx].first, keys[idx].second));
        sink += (it == map_index.end()) ? -1 : it->second;
      }
    }
    t_map_find = MsSince(t0);
  }
  {
    const auto t0 = Clock::now();
    for (int rep = 0; rep < rounds; ++rep) {
      for (size_t idx : order) {
        sink += vec_find(bf::Ident(keys[idx].first, keys[idx].second));
      }
    }
    t_vec_find = MsSince(t0);
  }

  const size_t n_lookups = static_cast<size_t>(rounds) * V;

  // ---- 3. Memory estimate (sizeof-based; excludes map bucket array overhead) ----
  // unordered_map node: key (Ident = 2x std::string = 64B) + value (int, padded)
  // + next ptr + hash cache ~= 88B/node plus a bucket pointer array (~1.3x load).
  const double map_mb =
      (V * (sizeof(bf::Ident) + sizeof(int) + 3 * sizeof(void*)) + V * sizeof(void*) * 13 / 10) /
      1048576.0;
  const double vec_mb = (vec_index.capacity() * sizeof(vec_index[0])) / 1048576.0;

  std::printf("sizeof: Ident=%zu FixedIdent=%zu FixedName8=%zu pair<FI,int>=%zu\n\n",
              sizeof(bf::Ident), sizeof(bf::FixedIdent), sizeof(FixedName8), sizeof(vec_index[0]));
  std::printf("%-22s %14s %14s\n", "", "unordered_map", "sorted vector");
  std::printf("%-22s %12.1f ms %12.1f ms\n", "build", t_map_build, t_vec_build);
  std::printf("%-22s %10.1f ns %12.1f ns   (%zu lookups)\n", "lookup (per op)",
              t_map_find * 1e6 / n_lookups, t_vec_find * 1e6 / n_lookups, n_lookups);
  std::printf("%-22s %11.1f MB %13.1f MB\n", "est. memory", map_mb, vec_mb);
  std::printf("\nlookup speedup vec/map: %.2fx   memory saving: %.1f MB   (sink=%lld)\n",
              t_map_find / t_vec_find, map_mb - vec_mb, static_cast<long long>(sink));
  return 0;
}
