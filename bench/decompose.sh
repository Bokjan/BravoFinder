#!/usr/bin/env bash
#
# Reproduce the "optimization decomposition" row of docs/performance.zh-CN.md:
# the same workload timed against three snapshots of the K-shortest search
# (baseline / +memoize / +Lawler).
#
# The three snapshots differ only in the search layer
# (core/graph/astar.* + core/graph/yen_kshortest.*). Rather than checking those
# files out of git history -- fragile: short hashes drift, the checkout dirties
# the working tree and index -- the exact source of each version is vendored,
# and checked in, under bench/variants/<v>/. This script just points a separate
# build tree at each via -DBRAVOFINDER_BENCH_VARIANT and rebuilds the benchmark.
#
# Nothing in the main source tree or build tree is touched: each variant gets
# its own build/bench-<v>/ directory.
#
# Usage:  bench/decompose.sh /path/to/nav.bfdb [rounds]
# Run from the repository root.

set -euo pipefail

DB="${1:-navdata/bfdb/nav.bfdb}"
ROUNDS="${2:-30}"

if [[ ! -f "$DB" ]]; then
  echo "cache not found: $DB" >&2
  echo "build one first:  ./build/release/apps/cli/bf build navdata -o /tmp/nav.bfdb" >&2
  exit 1
fi

REPO="$(git rev-parse --show-toplevel)"
cd "$REPO"

# label : one-line description. Sources live in bench/variants/<label>/.
STATES=(
  "baseline:Yen without heuristic memoization or Lawler"
  "memoize:multi-goal heuristic memoized across spur searches"
  "lawler:Lawler's optimization on top"
  "workspace:generation-stamped search workspace reused across spurs (workspace-reuse stage)"
)

for state in "${STATES[@]}"; do
  label="${state%%:*}"
  desc="${state#*:}"
  builddir="build/bench-${label}"

  cmake -S . -B "$builddir" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBRAVOFINDER_BUILD_BENCH=ON \
        -DBRAVOFINDER_BENCH_VARIANT="$label" >/dev/null
  cmake --build "$builddir" --target bf_route_bench >/dev/null

  echo "===== ${label}  ${desc} ====="
  "$builddir/bench/bf_route_bench" "$DB" "$ROUNDS"
  echo
done
