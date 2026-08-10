# BravoFinder

[![CI](https://github.com/Bokjan/BravoFinder/actions/workflows/ci.yml/badge.svg?branch=v3)](https://github.com/Bokjan/BravoFinder/actions/workflows/ci.yml) [![release](https://img.shields.io/github/v/tag/Bokjan/BravoFinder)](https://github.com/Bokjan/BravoFinder/releases) [![license](https://img.shields.io/badge/license-MIT%20%2F%20LGPL--3.0-blue)](LICENSE.md) ![C++20](https://img.shields.io/badge/C%2B%2B-20-blue) ![sanitizers](https://img.shields.io/badge/sanitizers-ASan%20%7C%20UBSan%20%7C%20TSan-red)

A realistic, fully-compliant flight-route engine in modern C++20 — plus a CLI, an MCP server, and a REST server built on top of it.

## Features

- **It's a library first.** `libs/engine/` is a self-contained C++20 static library (`bf::bravofinder`) with no JSON or network dependency — embed it in any host app. The CLI, MCP server, and REST server are just consumers of it.
- **Routes that read like filed flight plans.** v3 doesn't draw the geographic shortest path; it respects airway directionality, high/low airway levels, segment altitude bands, and terminal procedures (SID/STAR/approach).
- **Pluggable navigation data.** One `Loader` interface abstracts the source format — X-Plane 12 `.dat` (default), DFD SQLite (`dfd1`/`dfd2`), and Fenix A320. Support a new format without touching the engine.
- **Fast startup.** Compile a portable, little-endian binary cache (`.bfdb`) once per AIRAC cycle; subsequent loads are near-instant.

## Library

BravoFinder is fundamentally a route-engine **library**; the CLI, MCP server, and REST server are front-ends built on `bf::bravofinder`. If you only want routing inside your own app, you don't need to build the front-ends. Each release publishes a prebuilt SDK — consume `bf::bravofinder` via `find_package(bravofinder)` or `FetchContent`, no source build required. The public entry point is `bf::NavDatabase` (`libs/engine/io/nav_database.h`).

## Quick start

Requires C++20 and CMake 3.21+. Dependencies are fetched automatically via FetchContent.

```bash
cmake --preset release && cmake --build --preset release -j 32
ctest --preset release -j 32
```

Build a binary cache for your AIRAC cycle, then find a route (you supply the navigation data — see [Navigation data](#navigation-data)):

```bash
bf build navdata              # writes navdata/nav_<cycle>.bfdb
bf route KJFK KLAX            # reads ./navdata by default
bf route EGLL LFPG --format json --level high -k 3
```

## Navigation data

Navigation data is **not** included. It is copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only, and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## Usage

### CLI (`bf`)

The CLI is the primary front-end. `bf build` compiles your navigation data into a portable `.bfdb` cache for fast startup; `bf route` finds routes; `bf query` looks up navigation data; `bf parse-route` validates a filed route string.

```bash
# Build a cache once per AIRAC cycle (the source loader is auto-detected)
bf build navdata
bf build navdata --loader xplane12   # xplane12 | dfd1 | dfd2 | fenix

# Find routes
bf route KJFK KLAX
bf route KJFK KLAX --alt 300-400 --level high -k 3
bf route KJFK KLAX --db navdata/nav_2601.bfdb

# Validate a filed route string (reverse of route)
bf parse-route "KJFK SID CANDR Q480 HOTEE J80 MCI ... STAR KLAX" --db navdata/nav_2601.bfdb

# Look up navigation data (batch ids; JSON emits a parallel array)
bf query waypoint --db navdata/nav_2601.bfdb NINOX DGC
bf query airport  --db navdata/nav_2601.bfdb KJFK KLAX
```

Endpoints are airport ICAO codes or waypoint idents. When procedure data is present, routes name the SID/STAR used and show `SID`/`STAR` connectors; `--format json` adds per-leg distances and an ordered `points[]` array. Run `bf route --help` / `bf query --help` for the full option list (runways, SID/STAR selection, via/avoid points, reproducible `--seed`, cache load mode).

### MCP server (`bf-mcp`)

Exposes `bf route` and `bf query` as MCP tools for LLM clients, over **stdio** (default) or **HTTP** (`--transport http`, Streamable HTTP 2025-03-26). It serves a directory of `.bfdb` caches and opens multiple AIRAC cycles lazily; every tool takes an optional `cycle`. Tools: `find_routes`, `parse_route`, the `lookup_*` batch lookups, `lookup_procedure_legs`, and `list_cycles`. Full reference and client config: [apps/mcp/README.md](apps/mcp/README.md).

```bash
BRAVOFINDER_NAVDATA=navdata bf-mcp                       # stdio, local client
bf-mcp --transport http --db-dir /path/to/caches --port 8080
```

### HTTP server (`bf-http`)

A REST+JSON query server for a gateway to call over the network, sharing the `.bfdb`-directory model and transport core with `bf-mcp`. Per-endpoint request/response contract and status codes: [apps/http/README.md](apps/http/README.md). Design notes: [docs/http-service.zh-CN.md](docs/http-service.zh-CN.md).

## Building

Build a subset with `--target`, e.g. `cmake --build --preset release --target bf_mcp`. Targets:

| Target | Builds |
|---|---|
| `bf` | CLI tool |
| `bf_mcp` | MCP server (stdio / HTTP) |
| `bf_http` | HTTP query server |
| `bf_service_lib` | Shared service layer (`bf::service`) |
| `bf_tests` | Test runner |
| `bravofinder` | Unified engine static library (`bf::bravofinder`) |

A `tsan` preset verifies concurrency safety: `cmake --preset tsan && cmake --build --preset tsan -j 32 && ctest --preset tsan -j 32`.

## Contributors

[![Contributors](https://contrib.rocks/image?repo=Bokjan/BravoFinder)](https://github.com/Bokjan/BravoFinder/graphs/contributors)

## License

Dual-licensed: the engine (`libs/engine/`) is **LGPL-3.0-or-later** ([`libs/engine/LICENSE`](libs/engine/LICENSE)); everything else is **MIT** ([`LICENSE.MIT`](LICENSE.MIT)). Third-party licenses: [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md). The engine is statically linked into `bf`/`bf-http`/`bf-mcp`; under the LGPL, modifying it means you may relink, and the complete corresponding source ships with every release. Contributing: [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
