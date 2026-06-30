# BravoFinder

> **English** | [简体中文](README.zh-CN.md)

A flight route finder written in modern C++. **This is version 3 — a complete rewrite,
currently under active development.**

## About

BravoFinder builds a graph from navigation data (waypoints, navaids, airways, and
SID/STAR/approach procedures) and finds routes between two airports. Unlike earlier
versions, which computed a purely geographic shortest path, v3 aims to be a
**realistic / compliant route engine**: routes respect real-world constraints such as
airway directionality, high/low airway levels, segment altitude bands, and terminal
procedures.

## Status

v3 is being rewritten from scratch and is under active development.

**Working today (milestone M1):** routing over the enroute airway network. The tool
loads X-Plane 12 navigation data, builds a directed graph honoring airway
directionality, and finds the shortest path between two airports (or waypoints) with
A*. For example, `KJFK KLAX` resolves to a plausible ~2161 NM airway route.

### Roadmap

- **M1 (done)** — enroute airway network, A* search, `bf route` CLI.
- **M2** — pluggable constraints: airway direction, altitude bands, high/low level,
  MORA; Yen K-shortest for multiple candidate routes.
- **M3** — SID/STAR/approach procedures (ARINC 424 / CIFP), MSA.
- **M4** — procedure leg refinement and a compact `.bfdb` cache for instant startup.

Not planned for the first phase: Web API, map visualization.

## Building

Requires a C++20 compiler and CMake (3.21+). Dependencies (Catch2, CLI11) are fetched
automatically via FetchContent.

```bash
cmake --preset debug      # or: release
cmake --build --preset debug
ctest --preset debug      # runs unit tests; integration tests need data (see below)
```

## Usage

```bash
# Find a route (reads navigation data from ./navdata by default)
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KSEA KBOS --data /path/to/xplane/data
```

Endpoints are airport ICAO codes or waypoint idents, case-insensitive.

## Navigation Data

Navigation data is **not** included and must be supplied by the user. It is
copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only,
and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## License

[MIT](LICENSE). Third-party dependencies: see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
Contributing conventions: see [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
