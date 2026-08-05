# BravoFinder Documentation

> User-facing introduction and CLI usage live in the repository-root [README.md](../README.md). This is **technical documentation for readers** who want to understand what BravoFinder does and how it works inside. Articles are written in Chinese (`*.zh-CN.md`).

## Getting started

New to graph algorithms? Start with this primer — every other topic builds on its concepts:

- **[Route-finding basics](routing-basics.zh-CN.md)** — graphs / shortest paths / A\* (great-circle distance heuristic) / Yen K-shortest, from zero: how BravoFinder finds a route.

## Design highlights

In the order of "what → how it is modeled → how it is fast → how it is concurrent → how fast it measures":

- **[The compliant routing engine](compliant-routing.zh-CN.md)** — why not geographic shortest: airway directionality, high/low altitude layers, altitude bands, MORA, the pluggable constraint framework, Yen multi-candidate selection.
- **[The pluggable constraint layer](constraint-layer.zh-CN.md)** — the constraint framework's interface and discipline: three-way verdicts (Allow/Block/Penalize), why soft penalties do not break A\* admissibility, integer-only hot-path operations (sorted vectors / bitmasks), the parser/constraint layering; and the genuinely hard part of extension — "match granularity" (the same country rule read at name/instance/leg level, with a 7.5× false-hit spread) and "match semantics follow the data shape".
- **[Terrain safety: MORA grid and MSA sectors](terrain-safety.zh-CN.md)** — modeling two kinds of minimum safe altitude: MORA's 1° global dense grid vs MSA's per-airport sparse sectors, why two structures, and how each participates in routing.
- **[Procedure modeling and airway-network hookup](procedure-modeling.zh-CN.md)** — all 23 ARINC 424 / CIFP path terminators, real-SID/STAR airport hookup, junction-fix selection, multi-source K-shortest, DCT-to-IAF when no STAR exists, semantic honesty about connection kinds.
- **[Route-string compression](route-string.zh-CN.md)** — producing ICAO filed-flight-plan compliant compressed route strings: cumulative-intersection folding of confluent airways (`V28-Y28`), dodging the string-equality trap that fabricates fake transition points.
- **[Yen's Lawler optimization](yen-lawler-optimization.zh-CN.md)** — heuristic memoization + Lawler, ~2.5× faster K-shortest with identical results; includes correctness argument and differential spot-check verification of the non-standard multi-source variant.
- **[Cross-platform binary cache](binary-cache.zh-CN.md)** — the unified `.bfdb`: why explicit fixed-width little-endian instead of mmap, the global string pool, container/codec layering, on-demand / eager loading, graceful corruption errors.
- **[Thread-safety contract](thread-safety.zh-CN.md)** — concurrent queries on one read-only instance: double-checked locking, pointer stability across rehash, frozen lock-free eager reads, tsan verification.
- **[Domain modeling and memory design](domain-design.zh-CN.md)** — value types, the home-grown `Result<T,E>`, the design constitution (no `static` / no bare `new`), data-driven compact memory representations (`SmallVec`→`FixedIdent`); the v2→v3 rewrite's motivation made concrete.
- **[HTTP query service](http-service.zh-CN.md)** — `bf-http` REST and `bf-mcp --transport http` (Streamable HTTP): why serving beats in-process binding, the neutral `bf::service` and the shared `libs/http_server/` transport core, hand-rolled libuv + llhttp, the offload threading model and connection lifetime guard, HTTP security hardening, the status-code error model, MCP `Dispatcher` and batch/session semantics.
- **[Performance](performance.zh-CN.md)** — test machine config, methodology, ~11× startup, the optimization breakdown table, memory and cache size, reproducible steps.

## Contributing

- **[CONTRIBUTING.md](CONTRIBUTING.md)** — commit conventions (Conventional Commits), language rules, code style, version discipline, testing requirements.
