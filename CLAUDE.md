# BravoFinder — AI Agent Developer Guide

> **This file is for AI agents working in this repo.** It records hard invariants and
> rules that must survive across sessions — self-contained, no need to read the design
> doc to follow them. **User-facing** introduction and usage lives in `README.md` (do
> not move user-facing content into this file); this file is strictly "how to change
> code correctly."
>
> **Doc map** (each has one job; don't mix them):
> - `README.md` — user-facing: what the project is, how to build, CLI usage, data compliance.
> - `CLAUDE.md` (this file) — AI-agent-facing: hard conventions, invariants, minefields.
> - `.notes/` — local working docs (gitignored, not committed): `README.md` is the directory
>   index, `design/design-archive-M0-M4.md` is the archived M0–M4 design snapshot (historical;
>   current architecture lives in `docs/`), `plans/`/`records/`/`research/` are historical plans
>   and records. Start from `.notes/README.md` when you need background.
> - `docs/` — public-facing docs (CONTRIBUTING, algorithm articles, architecture: binary-cache /
>   domain-design / thread-safety / performance; the authority for "how it works now"; committed).

One-line background (details in README / docs/): a realistic/compliant flight route
engine that parses X-Plane 12 native navigation data (including ARINC 424 procedures),
builds a directed graph, and finds candidate routes respecting aviation constraints via
A\* + Yen K-shortest.

## Language

- Conversations, comments, documentation, and commit messages default to **Simplified
  Chinese**; docs prefer Chinese.
- Exceptions (keep in original): code identifiers, existing code style, technical
  proper nouns/commands/APIs, and **code comments use English**.

## Code conventions

- Modern C++20; **Google C++ Style** (format base, clang-format `BasedOnStyle: Google`)
  + **C++ Core Guidelines** (semantic correctness).
- File names `snake_case`, extensions **`.h` / `.cc`**; namespace `bf`.
- Error handling uses the project's own `bf::Result<T, E>` (`lib/core/result.h`); expected
  failures (no route found, missing data) go through `Result`, exceptions are only for
  truly exceptional situations. **No bare `new`/`delete`, no `goto`, no catch-by-value,
  no `static`/global mutable state** (v2's static-sharing bugs were one motivation for
  the rewrite).
- JSON output uses **RapidJSON `Writer`** (SAX streaming, auto-escaping); no hand-rolled
  strings, no nlohmann.

## Thread-safety Contract B (must follow when touching concurrency)

- After `NavDatabase::Open()` succeeds, the instance is **read-only**, with the sole
  exception of internal synchronized caches; `FindRoutes()` / `MsaForAirport()` are
  `const` and safe for concurrent calls on the same instance.
- On-demand: `procedure_cache_` is guarded by `cache_mutex_` (double-checked locking),
  only locking map lookup/insert, never disk I/O; append-only + `unique_ptr` values →
  returned pointers are stable across rehash.
- Eager: `FetchAll` fills the cache at `Open` time then **freezes** it; `ProceduresFor`
  does lock-free reads (no insert = no rehash = no race).
- **Eager mode is "frozen, lock-free reads" — it is not missing locks. Do not flag it
  in audits.**
- After touching concurrency-related code, **must pass the tsan preset**: `ctest
  --preset tsan`.

## Version discipline (three layers)

- ① Program semver (CMake `project VERSION` → `lib/core/version.h.in`'s
  `kBravoFinderVersion` → `bf --version`).
- ② Unified container `format_version` (one single version, magic "BFDB"; mismatch →
  `Result::Err(kFormatMismatch)`). Three sections (graph/cifp/detail) share one file and
  one version; no per-section version.
- ③ Program semver + source_loader + AIRAC cycle are written into the container header
  as provenance (`build` removed — it was an X-Plane-only field).
- **Cache disk layout change → bump container `format_version`; release behavior change
  → bump program semver.** Read-side / internal-only changes do not change the layout
  and do not bump.

## Testing

- **Real data, no mocks.** Real AIRAC data lives under `navdata/` (gitignored), located
  via `BRAVOFINDER_NAVDATA` (default `navdata/`); **missing data → `SKIP`, never mock
  as a substitute.**
- Unit tests use hand-constructed minimal real-format samples (not mock objects).
- For airports without CIFP procedures, test with **KIKR / KNWL**.
- Real navigation data / `*.dat` / `*.bfdb` **must never be committed** (Jeppesen
  copyright, redistribution prohibited).

## Build / test (during development)

> User-facing CLI usage (`bf build` / `bf route` arguments) lives in README; this
> section only lists the presets you must run after making changes. **Always use
> `--preset`, never the path form.**
```
cmake --preset debug      # Debug + ASan/UBSan + warnings-as-errors
cmake --preset release    # Release -O2
cmake --preset tsan       # ThreadSanitizer (concurrency verification)
cmake --build --preset <debug|release|tsan>
ctest --preset <debug|release|tsan>
```
On Windows MSVC, `windows-debug` / `windows-release` presets are available (no
sanitizers; used in CI).
Dependencies are pure CMake + FetchContent (Catch2 v3 / CLI11 / RapidJSON); no vendoring,
no vcpkg.

### Pick test scope by change (saves time; see Testing section for the principle)

ctest runs each case in its own process. Cache round-trip tests (`bfdb:` / `cifp
section:`, 11 cases) each take 20–40s and dominate wall-clock time — only run them when
**the cache disk layout / serialization** changed. For day-to-day work, pick by scope:
```
ctest --preset unit    # pure logic unit tests (~1.4s): algorithm/constraint/pure-function changes only
ctest --preset quick   # exclude cache tests (~12s): routing/query/constraint changes but cache layout untouched
ctest --preset debug   # full suite (~56s): cache layout/serialization changed, or pre-release
ctest --preset tsan    # must run after any concurrency-related change (Contract B)
```
(unit/quick are based on the debug config, with ASan/UBSan; they just use `filter` to
exclude the slow cache tests.)

## Git

- Commits use English **Conventional Commits**; body paragraphs are single unwrapped
  lines; keep the Claude co-author signature.
- For complex milestones, **align on the approach first, then implement**; at the end
  of each major task, do a docs / memory handoff.
