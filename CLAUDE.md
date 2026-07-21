# BravoFinder — AI Agent Developer Guide

> **This file is for AI agents working in this repo.** It records hard invariants and rules that must survive across sessions — self-contained, no need to read the design doc to follow them. **User-facing** introduction and usage lives in `README.md` (do not move user-facing content into this file); this file is strictly "how to change code correctly."

> **Doc map** (each has one job; don't mix them):
> - `README.md` — user-facing: what the project is, how to build, CLI usage, data compliance.
> - `CLAUDE.md` (this file) — AI-agent-facing: hard conventions, invariants, minefields.
> - `.notes/` — local working docs (gitignored, not committed): `README.md` is the directory index, `design/design-archive-M0-M4.md` is the archived M0–M4 design snapshot (historical; current architecture lives in `docs/`), `plans/`/`records/`/`research/` are historical plans and records. Start from `.notes/README.md` when you need background.
> - `docs/` — public-facing docs (CONTRIBUTING, algorithm articles, architecture: binary-cache / domain-design / thread-safety / http-service / performance; the authority for "how it works now"; committed).

One-line background (details in README / docs/): a realistic/compliant flight route engine that parses X-Plane 12 native navigation data (including ARINC 424 procedures), builds a directed graph, and finds candidate routes respecting aviation constraints via A\* + Yen K-shortest.

## Language

- Conversations, comments, documentation, and commit messages default to **Simplified Chinese**; docs prefer Chinese.
- Exceptions (keep in original): code identifiers, existing code style, technical proper nouns/commands/APIs, and **code comments use English**.
- **Markdown is never hard-wrapped (iron rule).** Every paragraph — in `README.md`, `docs/*.md`, `.notes/*.md`, and GitHub release notes — is **one logical line**; let the renderer soft-wrap. No manual column-80 folding inside a paragraph. Hard-wrapped paragraphs render as broken/odd line breaks when synced to GitHub releases (the `.notes/changelog/*.md` → `gh release edit` flow), and they make diffs noisy. This is the same discipline as git commit-message bodies (see Git). Code fences, tables, and ASCII/box diagrams are pre-formatted — leave their line breaks intact.

## Code conventions

- Modern C++20; **Google C++ Style** (format base, clang-format `BasedOnStyle: Google`)
  + **C++ Core Guidelines** (semantic correctness).
- File names `snake_case`, extensions **`.h` / `.cc`**; namespace `bf`.
- Error handling uses the project's own `bf::Result<T, E>` (`lib/core/result.h`); expected failures (no route found, missing data) go through `Result`, exceptions are only for truly exceptional situations. **No bare `new`/`delete`, no `goto`, no catch-by-value, no `static`/global mutable state** (v2's static-sharing bugs were one motivation for the rewrite).
- JSON output uses **RapidJSON `Writer`** (SAX streaming, auto-escaping); no hand-rolled strings, no nlohmann.

## Apps, transports, and the service layer (service / MCP / HTTP / CLI)

- The route engine is `lib/` (`bravofinder`, alias `bf::bravofinder`) and has **no** JSON / network dependency. Query capabilities shared across transports and the CLI live in the top-level `service/` (namespace **`bf::service`**, target `bf_service_lib`): the multi-cycle `NavDatabaseRegistry`, the typed query entries (`queries.h`), and the nine JSON-args handlers, each returning `HandlerResult{body, status}` (an HTTP-style code). Rendering (JSON + text) lives in `render.{h,cc}`; the handlers are a thin JSON-args adapter over the typed entries (`OutputFormat::kJson`), and the CLI calls the typed entries directly with the user's `--format` choice. `service/` is an app-tier library (it pulls in rapidjson), kept out of `lib/` so the engine stays JSON-/network-free.
- Three **peer** consumers depend on `service/`, never on each other: `apps/mcp` (`bf-mcp`, MCP over stdio — projects `is_error = status >= 400`), `apps/http` (`bf-http`, HTTP+JSON via libuv + llhttp — uses `status` directly), and `apps/cli` (`bf` — prints `body`, maps `status >= 400` to a non-zero exit code, uses the registry-free single-`NavDatabase` path). Add a query capability **once** in `bf::service`; all three get it. Don't add an http↔mcp↔cli dependency.
- **HTTP offload minefield** (`apps/http`): the 10–30 ms route compute must run on the libuv **threadpool** (`uv_queue_work`), never the loop thread. A work item holds a strong `shared_ptr<Connection>` so the connection survives a client disconnect mid- compute; the completion callback checks `IsAlive()` before writing, and workers never touch libuv handles. Touching this concurrency → `ctest --preset tsan` (Contract B).

## Thread-safety Contract B (must follow when touching concurrency)

- After `NavDatabase::Open()` succeeds, the instance is **read-only**, with the sole exception of internal synchronized caches; `FindRoutes()` / `MsaForAirport()` are `const` and safe for concurrent calls on the same instance.
- On-demand: `procedure_cache_` is guarded by `cache_mutex_` (double-checked locking), only locking map lookup/insert, never disk I/O; append-only + `unique_ptr` values → returned pointers are stable across rehash.
- Eager: `FetchAll` fills the cache at `Open` time then **freezes** it; `ProceduresFor` does lock-free reads (no insert = no rehash = no race).
- **Eager mode is "frozen, lock-free reads" — it is not missing locks. Do not flag it in audits.**
- After touching concurrency-related code, **must pass the tsan preset**: `ctest --preset tsan`.

## Version discipline (three layers)

- ① Program version (CMake `project VERSION` → `lib/core/version.h.in`'s `kBravoFinderVersion` → `bf --version`). Shape is MAJOR.MINOR.PATCH but the policy is **not** strict SemVer: v3 is in active development with interfaces unfrozen, so MAJOR stays pinned at 3 (absent a major shift), a large new feature or a breaking fix bumps MINOR, and any other code change / bugfix / tweak bumps PATCH.
- ② Unified container `format_version` (one single version, magic "BFDB"; mismatch → `Result::Err(kFormatMismatch)`). Three sections (graph/cifp/detail) share one file and one version; no per-section version.
- ③ Program version + source_loader + AIRAC cycle are written into the container header as provenance (`build` removed — it was an X-Plane-only field).
- **Cache disk layout change → bump container `format_version`; release behavior change → bump program version.** Read-side / internal-only changes do not change the layout and do not bump.

## Testing

- **Real data, no mocks.** Real AIRAC data lives under `navdata/` (gitignored), located via `BRAVOFINDER_NAVDATA` (default `navdata/`); **missing data → `SKIP`, never mock as a substitute.**
- Unit tests use hand-constructed minimal real-format samples (not mock objects).
- For airports without CIFP procedures, test with **KIKR / KNWL**.
- Real navigation data / `*.dat` / `*.bfdb` **must never be committed** (Jeppesen copyright, redistribution prohibited).

## Build / test (during development)

> User-facing CLI usage (`bf build` / `bf route` arguments) lives in README; this section only lists the presets you must run after making changes. **Always use `--preset`, never the path form.**
```
cmake --preset debug      # Debug + ASan/UBSan + warnings-as-errors
cmake --preset release    # Release -O2
cmake --preset tsan       # ThreadSanitizer (concurrency verification)
cmake --build --preset <debug|release|tsan>
ctest --preset <debug|release|tsan>
```
On Windows MSVC, `windows-debug` / `windows-release` presets are available (no sanitizers; used in CI). Dependencies are pure CMake + FetchContent (Catch2 v3 / CLI11 / RapidJSON; + libuv / llhttp for the HTTP server); no vendoring, no vcpkg.

### Pick test scope by change (saves time; see Testing section for the principle)

ctest runs each case in its own process. Cache round-trip tests (`bfdb:` / `cifp section:`, 11 cases) each take 20–40s and dominate wall-clock time — only run them when **the cache disk layout / serialization** changed. For day-to-day work, pick by scope:
```
ctest --preset unit    # pure logic unit tests (~1.4s): algorithm/constraint/pure-function changes only
ctest --preset quick   # exclude cache tests (~12s): routing/query/constraint changes but cache layout untouched
ctest --preset debug   # full suite (~56s): cache layout/serialization changed, or pre-release
ctest --preset tsan    # must run after any concurrency-related change (Contract B)
```
(unit/quick are based on the debug config, with ASan/UBSan; they just use `filter` to exclude the slow cache tests.)

## Git

- Commits use English **Conventional Commits**; body paragraphs are single unwrapped lines (same no-hard-wrap discipline as all markdown — see Language); keep the Claude co-author signature.
- For complex milestones, **align on the approach first, then implement**; at the end of each major task, do a docs / memory handoff.
