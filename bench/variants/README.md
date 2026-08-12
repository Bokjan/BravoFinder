# Bench search variants

The trees under `bench/variants/{baseline,memoize,lawler,workspace}/` are **historical frozen snapshots** of `astar.*` / `yen_kshortest.*` from the commits named in `docs/performance.zh-CN.md`. They exist so `bench/decompose.sh` can rebuild those four algorithm stages side-by-side without checking out git history.

Do **not** treat a variants binary as “current HEAD performance.” Remeasure today’s tree with `bench/route_bench` (or wall-clock `bf route`) on a release build; only use variants when you need the historical decomposition table itself.
