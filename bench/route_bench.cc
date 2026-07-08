// In-process microbenchmark for the K-shortest route search.
//
// Isolates pure search cost from process startup and cache loading: it opens the
// database ONCE, warms the on-demand procedure cache, then times only the
// FindRoutes calls with a steady_clock. Reports mean ms/search for k = 1/3/5/10
// over a fixed workload of 10 real city pairs.
//
// This is the tool behind the "查询耗时与优化分解" table in
// docs/performance.zh-CN.md. To reproduce the optimization decomposition across
// the baseline / +memoize / +Lawler variants, use bench/decompose.sh, which
// builds this binary against the vendored search snapshots under
// bench/variants/. See bench/README.md.
//
// Not part of the default build; enable with -DBRAVOFINDER_BUILD_BENCH=ON.

#include <chrono>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace {

// Ten real city pairs spanning different distances and graph topologies, so the
// mean is not dominated by one route's shape.
const std::vector<std::pair<std::string, std::string>> kPairs = {
    {"KJFK", "KLAX"}, {"KSEA", "KBOS"}, {"KDEN", "KSFO"}, {"KORD", "KDFW"}, {"KATL", "KLAS"},
    {"KMIA", "KSEA"}, {"KEWR", "KSAN"}, {"KIAH", "KPDX"}, {"KPHX", "KMSP"}, {"KDTW", "KSLC"}};

}  // namespace

int main(int argc, char** argv) {
  const std::string db = argc > 1 ? argv[1] : "navdata/nav.bfdb";
  const int rounds = argc > 2 ? std::atoi(argv[2]) : 30;

  // Optional altitude filter, e.g. "300-400" or "350". When given, every search
  // carries a cruise band, exercising the band + MORA constraints (the latter
  // samples every leg). Comparing a run with this against a run without it
  // isolates the net cost of altitude-filtered routing. Absent => unconstrained
  // shortest path, matching the numbers in docs/performance.zh-CN.md.
  std::optional<bf::FlRange> altitude;
  if (argc > 3) {
    int lo = 0;
    int hi = 0;
    if (std::sscanf(argv[3], "%d-%d", &lo, &hi) == 2) {
      altitude = bf::FlRange{lo, hi};
    } else if (std::sscanf(argv[3], "%d", &lo) == 1) {
      altitude = bf::FlRange{lo, lo};
    } else {
      std::fprintf(stderr, "bad alt spec %s (want \"300-400\" or \"350\")\n", argv[3]);
      return 1;
    }
  }

  bf::Result<bf::NavDatabase> nav = bf::NavDatabase::OpenCached(db);
  if (!nav) {
    std::fprintf(stderr, "cannot open %s: %s\n", db.c_str(), nav.error().message.c_str());
    std::fprintf(stderr, "usage: %s [nav.bfdb] [rounds] [alt e.g. 300-400]\n", argv[0]);
    return 1;
  }

  // Warmup: fill the on-demand procedure cache for every pair once, so the timed
  // loop measures search, not first-touch CIFP segment fetches.
  for (const auto& p : kPairs) {
    bf::RouteRequest r;
    r.departure = p.first;
    r.arrival = p.second;
    r.altitude = altitude;
    r.k = 10;
    (void)nav.value().FindRoutes(r);
  }

  std::printf("db=%s  workload=%zu pairs x %d rounds  alt=%s\n", db.c_str(), kPairs.size(), rounds,
              argc > 3 ? argv[3] : "none");
  for (int k : {1, 3, 5, 10}) {
    const auto t0 = std::chrono::steady_clock::now();
    int n = 0;
    for (int rep = 0; rep < rounds; ++rep) {
      for (const auto& p : kPairs) {
        bf::RouteRequest r;
        r.departure = p.first;
        r.arrival = p.second;
        r.altitude = altitude;
        r.k = k;
        const auto res = nav.value().FindRoutes(r);
        (void)res;
        ++n;
      }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("k=%2d : %7.3f ms/search  (%d searches)\n", k, ms / n, n);
  }
  return 0;
}
