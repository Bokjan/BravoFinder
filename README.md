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

v3 is being rewritten from scratch. See the milestones below; the API, CLI, and data
format support are still evolving.

### Planned (first phase)

- **`core/`** — domain library: domain model, compact CSR graph, A* + Yen
  K-shortest, pluggable constraints. No global/static state; `bf::Result<T, E>` error
  handling.
- **`bf` CLI** — `bf build` (parse nav data and produce a compact `.bfdb` cache) and
  `bf route` (query candidate routes fast).
- **Data source** — X-Plane 12 native `.dat` (incl. ARINC 424 procedure parsing).

Not in the first phase: Web API, map visualization.

## Building

Requires a C++20 compiler and CMake. Dependencies (Catch2, CLI11) are fetched
automatically via FetchContent.

```bash
cmake --preset debug
cmake --build --preset debug
```

## Navigation Data

Navigation data is **not** included and must be supplied by the user. It is
copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only,
and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## License

[MIT](LICENSE). Third-party dependencies: see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
Contributing conventions: see [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
