# Performance benchmarks

Tooling and recipes behind the numbers in `docs/performance.zh-CN.md`, kept here
so the measurements can be reproduced on another machine or version. The numbers
and their interpretation live in that document; this file is only about *how to
run*.

Nothing here is part of the default build (`BRAVOFINDER_BUILD_BENCH` defaults to
OFF), so it never affects `bf`, the tests, or CI.

## 0. Prerequisites

```bash
cmake --preset release && cmake --build --preset release
# Build one unified cache that every benchmark reuses (rebuild only on AIRAC change).
./build/release/apps/cli/bf build navdata -o /tmp/nav.bfdb
```

## 1. Startup time: cold start vs. cache load (end-to-end wall clock)

Process-level wall clock (includes startup and teardown); run 5 times and take
the stable value:

```bash
BF=./build/release/apps/cli/bf
for i in $(seq 5); do /usr/bin/time -p $BF route KJFK KLAX --data navdata >/dev/null; done   # cold: parse + build graph
for i in $(seq 5); do /usr/bin/time -p $BF route KJFK KLAX --db /tmp/nav.bfdb >/dev/null; done # cache load
```

## 2. Peak resident memory (RSS)

```bash
# Linux: the "Maximum resident set size" line.
/usr/bin/time -v $BF route KJFK KLAX --db /tmp/nav.bfdb 2>&1 | grep 'Maximum resident'
/usr/bin/time -v $BF route KJFK KLAX --db /tmp/nav.bfdb --cifp-load eager 2>&1 | grep 'Maximum resident'
# macOS: use /usr/bin/time -l (field name "maximum resident set size").
```

## 3. Pure search time (`route_bench`)

Strips out startup noise: opens the database once in-process, warms the
on-demand procedure cache, then times only the `FindRoutes` calls with a
`steady_clock`. Reports mean ms/search for k = 1/3/5/10 over a fixed workload of
10 real city pairs.

```bash
cmake --preset release -DBRAVOFINDER_BUILD_BENCH=ON && cmake --build --preset release --target bf_route_bench
./build/release/bench/bf_route_bench /tmp/nav.bfdb            # 30 rounds by default
./build/release/bench/bf_route_bench /tmp/nav.bfdb 100        # optional: more rounds to cut noise
./build/release/bench/bf_route_bench /tmp/nav.bfdb 100 300-400  # optional: attach a cruise band (exercises the band + MORA constraints)
```

The optional third argument is a cruise altitude filter (`"300-400"` for a
band, `"350"` for a single level). It turns on the altitude-band and MORA
constraints on every search; comparing a run with it against a run without
isolates the net cost of altitude-filtered routing. Omit it for the
unconstrained shortest-path numbers in `docs/performance.zh-CN.md`.

### 3a. Optimization decomposition (baseline / +memoize / +Lawler)

The three variants differ only in the search layer
(`core/graph/astar.*` + `core/graph/yen_kshortest.*`). The exact source of each
is vendored, and checked in, under `bench/variants/<variant>/` -- so this needs
no `git checkout` of historical commits (short hashes drift, and checking files
out of history dirties the working tree and index). `bench/decompose.sh` points
a separate build tree at each variant via `-DBRAVOFINDER_BENCH_VARIANT` and runs
`route_bench` against the same cache:

```bash
./bench/decompose.sh /tmp/nav.bfdb        # 30 rounds by default
./bench/decompose.sh /tmp/nav.bfdb 100    # optional: more rounds
```

The variants and their provenance:

- `baseline` -- Yen with no heuristic memoization and no Lawler (commit `2918c86`).
- `memoize` -- multi-goal heuristic memoized across spur searches (commit `ee3afb4`).
- `lawler` -- Lawler's optimization on top; algorithm-equivalent to the current
  HEAD (commit `f7a42c9`, which differs from HEAD only by later clang-format).

Each build lands in its own `build/bench-<variant>/`; the main source tree and
`build/release/` are never touched. To time a single variant directly:

```bash
cmake -S . -B build/bench-baseline -DCMAKE_BUILD_TYPE=Release \
      -DBRAVOFINDER_BUILD_BENCH=ON -DBRAVOFINDER_BENCH_VARIANT=baseline
cmake --build build/bench-baseline --target bf_route_bench
./build/bench-baseline/bench/bf_route_bench /tmp/nav.bfdb
```

An empty `BRAVOFINDER_BENCH_VARIANT` (the default) builds the current tree
unchanged -- the variant machinery only activates when the option is set, which
happens only in these throwaway build trees, never in a normal build.

## 4. Profile (gprof, to pin the intrinsic hot spot)

gprof needs a `-pg` whole-program build, so it does not go through the CMake
libraries: compile `core/` + `io/` together with a minimal driver
(`-I build/<preset>/core` finds the generated `version.h`):

```bash
cat > /tmp/prof_drv.cc <<'CPP'
#include "core/routing/route_request.h"
#include "io/nav_database.h"
int main(int argc, char** argv){
  auto nav = bf::NavDatabase::OpenCached(argc>1?argv[1]:"navdata/nav.bfdb");
  if(!nav) return 1;
  bf::RouteRequest r; r.departure="KJFK"; r.arrival="KLAX"; r.k=10;
  (void)nav.value().FindRoutes(r);                 // warmup
  for(int i=0;i<400;++i){ auto x=nav.value().FindRoutes(r); (void)x; }
  return 0;
}
CPP
g++ -std=c++20 -O2 -pg -I. -Ibuild/release/core /tmp/prof_drv.cc \
    $(find core io -name '*.cc') -o /tmp/prof_drv
cd /tmp && ./prof_drv /path/to/nav.bfdb && gprof /tmp/prof_drv gmon.out | head -20
```

gprof folds the edge relaxation, g-value updates, and heap operations inlined
into the A* main loop into `RunMultiSearch`, and does not sample malloc or
syscalls, so its share is higher than a call-stack sampler (perf, macOS
`sample`) would report; the order-of-magnitude conclusion is the same: the A*
traversal itself is the dominant hot spot.

Note the vendored variant sources under `bench/variants/` intentionally
duplicate historical algorithm code -- that is the point (a self-contained,
git-history-independent snapshot). They are never compiled into `bf` or the
tests.
