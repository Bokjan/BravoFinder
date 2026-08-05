# BravoFinder

[![CI](https://github.com/Bokjan/BravoFinder/actions/workflows/ci.yml/badge.svg?branch=v3)](https://github.com/Bokjan/BravoFinder/actions/workflows/ci.yml) [![release](https://img.shields.io/github/v/tag/Bokjan/BravoFinder)](https://github.com/Bokjan/BravoFinder/releases) [![license](https://img.shields.io/badge/license-MIT%20%2F%20LGPL--3.0-blue)](LICENSE.md) ![C++20](https://img.shields.io/badge/C%2B%2B-20-blue) ![sanitizers](https://img.shields.io/badge/sanitizers-ASan%20%7C%20UBSan%20%7C%20TSan-red)

A realistic / compliant flight-route **engine library** (`libs/engine`) in modern C++20, with three front-ends built on it — a CLI, an MCP server, and a REST server.

## About

At its core, **BravoFinder is a library**: `libs/engine/` (namespace `bf`, CMake target `bf::bravofinder`) is a realistic / compliant flight-route engine with **no JSON or network dependency**. It parses navigation data into a directed graph and finds routes between two airports (or waypoints) that respect real-world constraints — airway directionality, high/low airway levels, segment altitude bands, and terminal procedures (SID/STAR/approach). Unlike earlier versions, which computed a purely geographic shortest path, v3 routes read like filed flight plans. The engine ships as a self-contained static library (`bf::bravofinder`) you can embed in any host application; the public entry point is `bf::NavDatabase` (`io/nav_database.h`).

Navigation data is read through a **pluggable `Loader` interface** (`libs/engine/io/loaders/`) that abstracts the source format. Four loaders ship today: `xplane12` (the default — X-Plane 12 native `.dat`), `dfd1` / `dfd2` for the DFD SQLite databases shipped by RealTraffic / SimToolkitPro / PMDG MSFS and Inibuilds A350 respectively, and `fenix` for the Fenix A320 navdata SQLite database. Adding a format means adding a loader; the route engine is untouched. The selected loader is recorded as `source_loader` provenance in every `.bfdb` cache header.

Three front-ends are built on the engine and ship in this repo — the `bf` CLI, an MCP server (`bf-mcp`) for LLM clients, and a REST+JSON server (`bf-http`) — sharing one service layer (`libs/service/`); see [Usage](#usage).

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
| `bf_mcp` | MCP server, stdio or HTTP (`apps/mcp/`) |
| `bf_http` | HTTP query server (`apps/http/`) |
| `bf_service_lib` | Shared service layer: registry + handlers + typed entries, `bf::service` (static) |
| `bf_tests` | Test runner |
| `bravofinder` | The unified static library (`libs/engine/`, alias `bf::bravofinder`) |

```bash
cmake --build --preset debug --target bf_mcp    # just the MCP server
```

A `tsan` preset (ThreadSanitizer) is available to verify concurrency safety:

```bash
cmake --preset tsan && cmake --build --preset tsan && ctest --preset tsan
```

A prebuilt SDK archive is published with each release — consume the `bf::bravofinder` target via `find_package(bravofinder)` or `FetchContent`.

## Usage

### MCP server (`bf-mcp`)

BravoFinder also ships an MCP server that exposes `bf route` and `bf query` as MCP tools, so an LLM client can ask for routes and look up navigation data directly. It speaks MCP over one of two transports: **stdio** (default — JSON-RPC over stdin/stdout, for a local client that spawns the process) or **HTTP** (`--transport http` — Streamable HTTP, 2025-03-26, on a TCP port, for remote / multi-client access). The stdio transport is zero-dependency beyond the project's own library; the HTTP transport shares the `bf-http` transport core (libuv + llhttp).

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

# Serve MCP over HTTP instead of stdio (Streamable HTTP, single endpoint /mcp):
bf-mcp --transport http --db-dir /path/to/caches --host 0.0.0.0 --port 8080
# HTTP-mode flags mirror bf-http: --worker-threads N, --max-body BYTES, --io-timeout SEC.
# --cifp-load on-demand|eager applies to both transports (eager = lock-free reads, ~100 MB/cycle).
```

Tools exposed: `find_routes` and `parse_route` (mirroring `bf route`), the `lookup_waypoints` / `lookup_airports` / `lookup_procedures` / `lookup_airways` / `lookup_navaid_detail` / `lookup_holds` batch lookups plus `lookup_procedure_legs` (a named procedure's per-leg detail; mirroring `bf query`), and `list_cycles`. See [apps/mcp/README.md](apps/mcp/README.md) for the full tool reference, argument semantics, and client configuration.

`bf build` (cache creation) remains a CLI concern and is not exposed as a tool.

### HTTP server (`bf-http`)

BravoFinder also ships an HTTP+JSON query server (REST-style endpoints) for a gateway to call over the network. It shares the MCP server's `.bfdb`-directory model — lazy per-cycle open, optional `?cycle=`, fails fast if no cache — and its transport core (`libs/http_server/`, over libuv + llhttp, with each route computation offloaded to a threadpool). The full per-endpoint request/response contract and status codes live in [apps/http/README.md](apps/http/README.md); the design is in [docs/http-service.zh-CN.md](docs/http-service.zh-CN.md).

### CLI (`bf`)

```bash
# Build a binary cache once per AIRAC cycle for fast startup.
bf build navdata                    # writes navdata/nav_<cycle>.bfdb
bf build navdata --loader xplane12  # source loader: xplane12 | dfd1 | dfd2 | fenix (default xplane12)

# Find a route (reads ./navdata by default)
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KJFK KLAX --db navdata/nav_2601.bfdb   # load a prebuilt cache

# Constrain the search
bf route KJFK KLAX --alt 300-400      # altitude band (enables MORA filtering)
bf route KJFK KLAX --level high -k 3  # prefer Jet airways; ask for 3 candidates

# Restrict airways by ICAO region + designator (repeatable; one value = one rule)
bf route ZSSS ZGGG --airway-filter='*:J60=block'   # forbid exactly J60, anywhere
bf route ZSSS ZGGG --airway-filter='ZB,ZG,ZH,ZJ,ZL,ZP,ZS,ZU,ZW,ZY:J*=block'
bf route VABB VIDP --airway-filter='VI,VA,VO,VE:J*=penalize:0.3'

# Validate a filed route string (reverse of route)
bf parse-route "KJFK SID CANDR Q480 HOTEE J80 MCI ... STAR KLAX" --db navdata/nav_2601.bfdb

# Look up navigation data (batch ids; --format json emits a parallel array)
bf query waypoint --db navdata/nav_2601.bfdb NINOX DGC
bf query airport  --db navdata/nav_2601.bfdb KJFK KLAX

# Print the program version
bf --version
```

More options — runways, SID/STAR selection, via/avoid points, reproducible `--seed`, and cache load mode (`--cifp-load eager|on-demand`) — are in `bf route --help` / `bf query --help`.

`--airway-filter` restricts airways by ICAO region and designator, because usage conventions are regional: a `J` route is a terminal transition in China but a legal Jet route in the US, and the source data carries no type field to tell them apart. The syntax is `<regions>:<designators>[=block|penalize[:<fraction>]]`, where a trailing `*` means prefix, no `*` means exact, a bare `*` means any, and commas list several. It replaces the old `--avoid-awy J60`, whose equivalent is `--airway-filter='*:J60=block'`.

Three things are worth knowing. **Matching is per leg, not per airway name**: a designator is not unique to one physical airway (29.6% of names in cycle 2601 are reused by disjoint instances, up to 18 for one name), so a name-level ban would forbid same-named airways worldwide — a China-wide `J` rule would also kill the legal US Jet routes. **Regions are prefix-matched, so `Z` is wider than "China"**: it also covers `ZM` (Mongolia) and `ZK` (North Korea), which is why the example above enumerates the ten mainland FIRs instead. **`block` can leave you with no route at all**: it is an irreversible connectivity break, and an airport whose only terminal connection is a blocked airway becomes unroutable — prefer `penalize` for bulk region rules, which keeps the graph connected and lets a shorter alternative win on cost.

Route endpoints are airport ICAO codes or waypoint idents, case-insensitive. When an airport has procedure data, the route names the SID and STAR used in its `sid`/`star` fields (and the interchangeable procedures that share the same connection fix); in the route string and the leg list they show as the literal `SID`/`STAR` connectors. Airports with no published STAR but with approaches use DCT-to-IAF: the filed string still ends `… <fix> DCT ARR`, `star` stays empty, and approach detail appears only as metadata (`arr_connection=terminal_transition` plus approach fields in JSON).

With `--format json`, route output carries the filed route string, per-phase distances, an ordered `points[]` array (each `{ident, lat, lon}`), and a running `cumulative_nm` on every leg.

The `.bfdb` cache is a portable, little-endian binary snapshot. One unified file holds three sections sharing a global string pool: the route graph, the per-airport CIFP procedures (loaded on demand), and the radio-navaid attributes and holding patterns for the `navaid_detail` / `hold` lookups. The canonical name is `nav_<cycle>.bfdb`, encoding the AIRAC cycle so a directory can hold several cycles. It is derived from Navigraph/Jeppesen data and, like the source data, must not be redistributed (it is git-ignored).

## Navigation Data

Navigation data is **not** included and must be supplied by the user. It is copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only, and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## Documentation

In-depth technical articles (in Chinese) live under [docs/](docs/README.md) — start with the routing-algorithm primer and follow the index from there.

## Contributors

[![Contributors](https://contrib.rocks/image?repo=Bokjan/BravoFinder)](https://github.com/Bokjan/BravoFinder/graphs/contributors)

## License

This project is dual-licensed. The core route engine library under `libs/engine/` is distributed under the **GNU Lesser General Public License v3.0-or-later** (see [`libs/engine/LICENSE`](libs/engine/LICENSE)); everything else is **MIT** (text in [`LICENSE.MIT`](LICENSE.MIT)). Third-party dependencies: see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md). Contributing conventions: see [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).

The `libs/engine` library is statically linked into the `bf` / `bf-http` / `bf-mcp` binaries. Under the LGPL, if you modify `libs/engine` you may relink it into those binaries; the complete corresponding source (including the engine) is published with every release and in this repository.
