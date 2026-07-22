# BravoFinder

[![CI](https://github.com/Bokjan/BravoFinder/actions/workflows/ci.yml/badge.svg?branch=v3)](https://github.com/Bokjan/BravoFinder/actions/workflows/ci.yml) [![release](https://img.shields.io/github/v/tag/Bokjan/BravoFinder)](https://github.com/Bokjan/BravoFinder/releases) [![license](https://img.shields.io/github/license/Bokjan/BravoFinder)](LICENSE) ![C++20](https://img.shields.io/badge/C%2B%2B-20-blue) ![sanitizers](https://img.shields.io/badge/sanitizers-ASan%20%7C%20UBSan%20%7C%20TSan-red)

A flight route finder written in modern C++ (v3).

## About

BravoFinder builds a graph from navigation data (waypoints, navaids, airways, and SID/STAR/approach procedures) and finds routes between two airports. Unlike earlier versions, which computed a purely geographic shortest path, v3 is a **realistic / compliant route engine**: routes respect real-world constraints such as airway directionality, high/low airway levels, segment altitude bands, and terminal procedures.

Navigation data is read through a **pluggable `Loader` interface** (`lib/io/loaders/`) that abstracts the source format. Three loaders ship today: `xplane12` (the default — X-Plane 12 native `.dat`), and `dfd1` / `dfd2` for the DFD SQLite databases shipped by RealTraffic / SimToolkitPro / PMDG MSFS and Inibuilds A350 respectively. Adding a format means adding a loader; the route engine is untouched. The selected loader is recorded as `source_loader` provenance in every `.bfdb` cache header.

## Status

The tool builds a directed graph honoring airway directionality and high/low levels, and finds routes between two airports (or waypoints) with A* and Yen K-shortest. Airports connect to the enroute network through their real SID/STAR procedures (parsed from ARINC 424 / CIFP), falling back to a direct link where no procedure data exists. For example, `KJFK KLAX` resolves to a filed-flight-plan-style route such as `KJFK SID TOWIN ... PGS STAR KLAX` of ~2160 NM (the literal `SID`/`STAR` connect the airports to the enroute network; the actual procedure names appear in the route's `sid`/`star` fields).

A single loaded database is safe to query concurrently from multiple threads.

The engine is exposed through three front-ends (see [Usage](#usage)): the `bf` CLI, an MCP stdio server (`bf-mcp`) for LLM clients, and an HTTP+JSON server (`bf-http`) for network callers. The latter two share one service layer (`service/`, namespace `bf::service`).

## Building

Requires a C++20 compiler and CMake (3.21+). Dependencies (Catch2, CLI11, RapidJSON, SQLite, plus libuv and llhttp for the HTTP server) are fetched automatically via FetchContent.

```bash
cmake --preset debug              # or: release
cmake --build --preset debug      # parallel build (use --preset, not the path form)
ctest --preset debug              # unit tests run always; integration tests need data
```

Build only what you need with `--target`:

| Target | What it builds |
|---|---|
| `bf` | CLI tool (`apps/cli/`) |
| `bf_mcp` | MCP stdio server (`apps/mcp/`) |
| `bf_mcp_lib`   | MCP server library (static) |
| `bf_http` | HTTP query server (`apps/http/`) |
| `bf_service_lib` | Shared service layer: registry + handlers + typed entries, `bf::service` (static) |
| `bf_tests` | Test runner |
| `bravofinder` | The unified static library (`lib/`, alias `bf::bravofinder`) |

```bash
cmake --build --preset debug --target bf_mcp    # just the MCP server
cmake --build --preset debug --target bf bf_http      # CLI + HTTP server
```

A `tsan` preset (ThreadSanitizer) is available to verify concurrency safety:

```bash
cmake --preset tsan && cmake --build --preset tsan && ctest --preset tsan
```

### Using the library (SDK)

The route engine ships as a self-contained static library. Two ways to consume it, both under the single target name `bf::bravofinder`:

**Pre-built SDK** — download a `bravofinder-sdk-*` archive from a [release](https://github.com/Bokjan/BravoFinder/releases), unpack it, and:

```cmake
find_package(bravofinder REQUIRED)
target_link_libraries(my_app PRIVATE bf::bravofinder)
```

The static archive (`libbravofinder.a` / `bravofinder.lib`) folds in the SQLite amalgamation, so no separate sqlite dependency is needed. On MSVC the SDK uses the default dynamic CRT (`/MD`); match that in the consuming project.

**From source (FetchContent)**:

```cmake
include(FetchContent)
FetchContent_Declare(
  BravoFinder
  GIT_REPOSITORY https://github.com/Bokjan/BravoFinder.git
  GIT_TAG v3)
FetchContent_MakeAvailable(BravoFinder)
target_link_libraries(my_app PRIVATE bf::bravofinder)
```

The public entry point is `bf::NavDatabase` (`#include "io/nav_database.h"`); headers are included as `core/...` / `io/...` rooted at `bf/`.

## Usage

### MCP server (`bf-mcp`)

BravoFinder also ships a local MCP server that exposes `bf route` and `bf query` as MCP tools over stdio, so an LLM client can ask for routes and look up navigation data directly. It is a thin, zero-dependency (beyond the project's own library) stdio JSON-RPC server.

It is pointed at a **directory** of `.bfdb` caches (not a single file) and can serve multiple AIRAC cycles from it: each cycle's database is opened lazily on first use and cached. Every tool takes an optional `cycle` argument (omit for the newest), and a `list_cycles` tool enumerates what is available. A single-cycle deployment is just a directory holding one cache.

Build it alongside the CLI:

```bash
cmake --preset release && cmake --build --preset release   # or: debug
# binary: build/release/apps/mcp/bf-mcp
```

Point it at a directory and run it (it fails fast at startup if the directory holds no `nav_<cycle>.bfdb` cache):

```bash
# The directory is --db-dir, else BRAVOFINDER_NAVDATA, else ./navdata.
BRAVOFINDER_NAVDATA=navdata bf-mcp
bf-mcp --db-dir /path/to/caches
```

Tools exposed: `find_routes` and `parse_route` (mirroring `bf route`), the `lookup_waypoints` / `lookup_airports` / `lookup_procedures` / `lookup_airways` / `lookup_navaid_detail` / `lookup_holds` batch lookups plus `lookup_procedure_legs` (a named procedure's per-leg detail; mirroring `bf query`), and `list_cycles`. See [apps/mcp/README.md](apps/mcp/README.md) for the full tool reference, argument semantics, and client configuration.

`bf build` (cache creation) remains a CLI concern and is not exposed as a tool.

### HTTP server (`bf-http`)

BravoFinder also ships an HTTP+JSON query server for an internal (e.g. Go) gateway to call over the network. It exposes the same route-finding and navigation-data lookups as the MCP server, but as REST-style endpoints. It is a hand-rolled transport over [libuv](https://github.com/libuv/libuv) (async I/O + a worker threadpool) and [llhttp](https://github.com/nodejs/llhttp) (Node's HTTP parser): a single event-loop thread does all non-blocking I/O, and each route computation is offloaded to the threadpool, so one loop scales to many connections.

Like the MCP server, it is pointed at a **directory** of `.bfdb` caches and can serve multiple AIRAC cycles (query endpoints accept an optional `?cycle=2601`, defaulting to the newest). It fails fast at startup if the directory holds no cache.

Build and run it:

```bash
cmake --preset release && cmake --build --preset release   # or: debug
# binary: build/release/apps/http/bf-http

# The directory is --db-dir, else BRAVOFINDER_NAVDATA, else ./navdata.
bf-http --db-dir /path/to/caches --host 0.0.0.0 --port 8080
# Other flags: --worker-threads N (threadpool size), --max-body BYTES,
# --io-timeout SEC (header/body read + idle keep-alive).
```

Endpoints (all query endpoints are `POST` with a JSON body; a batch lookup takes `{"ids":[...]}`, a single lookup is a one-element array):

| Method | Path | Purpose |
|---|---|---|
| POST | `/v1/routes` | find k candidate routes |
| POST | `/v1/parse-route` | validate/expand a filed route string |
| POST | `/v1/waypoints` `/v1/airports` `/v1/procedures` `/v1/airways` `/v1/navaid-detail` `/v1/holds` | batch lookups (parallel to `ids`) |
| POST | `/v1/procedure-legs` | one named procedure's per-leg detail |
| GET | `/v1/cycles` | list the servable AIRAC cycles |
| GET | `/healthz` `/readyz` | liveness / readiness probes |

Errors return `{"error":"..."}` with an HTTP status: **400** for a malformed request (bad JSON, missing/invalid field, bad `?cycle=`), **404** when nothing matched (all ids missing, or an unknown path), and **422** when a well-formed request cannot be satisfied (no route, a bad route token). Request bodies over `--max-body` get **413**; `Transfer-Encoding: chunked` is refused. See [apps/http/README.md](apps/http/README.md) for the full per-endpoint request/response contract, and [docs/http-service.zh-CN.md](docs/http-service.zh-CN.md) for the design.

`bf build` (cache creation) remains a CLI concern and is not exposed here.

### CLI (`bf`)

```bash
# Build a binary cache once per AIRAC cycle for fast startup (~2.3s -> ~0.2s).
# The default name encodes the cycle so a directory of caches can hold several
# AIRACs. One unified .bfdb holds the graph, the CIFP procedures, and the navaid
# detail, so deployment needs only that file, not the CIFP/ directory.
bf build navdata                    # writes navdata/nav_<cycle>.bfdb
bf build /path/to/xplane -o my.bfdb # explicit name: writes my.bfdb
bf build navdata --loader xplane12  # select source loader: xplane12 | dfd1 | dfd2 (default xplane12)

# Find a route (reads navigation data from ./navdata by default)
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KSEA KBOS --data /path/to/xplane/data

# Load a prebuilt cache to skip parsing. The one .bfdb carries the graph, the
# CIFP procedures, and the navaid detail, so the CIFP/ directory is not needed
# at all.
bf route KJFK KLAX --db navdata/nav_2601.bfdb

# Procedure cache load mode: on-demand (default, ~1.5 MB, best for one-shot
# queries) or eager (loads all procedures up front, ~100 MB then lock-free,
# best for servers / batch routing)
bf route KJFK KLAX --db navdata/nav_2601.bfdb --cifp-load eager

# Constrain by cruise altitude (enables altitude-band and MORA filtering).
# A single level or an inclusive range (any level in the band is acceptable).
bf route KJFK KLAX --alt 350
bf route KJFK KLAX --alt 300-400

# Prefer high (Jet) or low (Victor) airways; ask for several candidates
bf route KJFK KLAX --level high -k 3

# Restrict the departure/arrival runway used for SID/STAR selection
bf route KJFK KLAX --rwy-dep RW31L
bf route KJFK KLAX --rwy-arr RW25L

# Select a specific SID/STAR by name (bare name matches any transition;
# NAME.TRANSITION pins the transition). An unknown name is a clean error.
bf route KJFK KLAX --sid DEEZZ5
bf route KJFK KLAX --star LENDY6.HAAYS

# Force the route through waypoints, in order (via points); ident or IDENT/REGION.
bf route KJFK KLAX --via DBL
bf route KJFK KLAX --via PSB --via DBL

# Route around waypoints or airways. A bare ident avoids all its regional
# matches; an airway designator also blocks its concurrency segments.
bf route KJFK KLAX --avoid-wpt CANDR
bf route KJFK KLAX --avoid-awy J60

# Diversify the route reproducibly: the same seed always yields the same route,
# different seeds explore alternative (still valid) routes.
bf route KJFK KLAX --seed 42

# Validate and expand a filed route string (the reverse of route): checks that
# each airway connects its bracketing fixes, expands airways to their
# intermediate points, and totals the distance. Errors name the bad token.
bf parse-route "KJFK SID CANDR Q480 HOTEE J80 MCI ... STAR KLAX" --db navdata/nav_2601.bfdb

# Look up navigation data: waypoints, airports, procedures, airways, navaid
# details, or holds. Each accepts one or more ids (a batch), and --format json
# emits an array parallel to the input (a not-found id becomes null).
bf query waypoint --db navdata/nav_2601.bfdb NINOX DGC
bf query airport  --db navdata/nav_2601.bfdb KJFK KLAX
bf query procedure --db navdata/nav_2601.bfdb KJFK
bf query airway   --db navdata/nav_2601.bfdb Y28 --format json
# navaid_detail: frequency, service range, elevation, station variation/bearing.
bf query navaid_detail --db navdata/nav_2601.bfdb SEA DGC
# hold: holding-pattern parameters (inbound course, leg length, turn, altitude).
bf query hold --db navdata/nav_2601.bfdb AE701

# Print the program version
bf --version
```

Route endpoints are airport ICAO codes or waypoint idents, case-insensitive. When an airport has procedure data, the route names the SID and STAR used in its `sid`/ `star` fields (and the interchangeable procedures that share the same connection fix); in the route string and the leg list they show as the literal `SID`/`STAR` connectors.

The `--format json` route output carries, alongside the filed route string and per-phase distances, an ordered `points[]` array (each `{ident, lat, lon}`) and a running `cumulative_nm` on every leg. `points` has one more entry than `legs` (N points, N-1 legs); the destination of leg *i* is `points[i+1]`, and the last `cumulative_nm` equals `total_distance_nm`.

The `.bfdb` cache is a portable, little-endian binary snapshot. One unified file holds three sections sharing a global string pool: the route graph, the per-airport CIFP procedures (loaded on demand), and the radio-navaid attributes and holding patterns for the `navaid_detail` / `hold` lookups. The canonical name is `nav_<cycle>.bfdb`, encoding the AIRAC cycle so a directory can hold several cycles. It is derived from Navigraph/Jeppesen data and, like the source data, must not be redistributed (it is git-ignored).

## Navigation Data

Navigation data is **not** included and must be supplied by the user. It is copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only, and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## Documentation

In-depth technical articles (in Chinese) live under [docs/](docs/README.md) — start with the routing-algorithm primer and follow the index from there.

## License

[MIT](LICENSE). Third-party dependencies: see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md). Contributing conventions: see [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
