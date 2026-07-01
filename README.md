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

**Working today (through milestone M3):** the tool loads X-Plane 12 navigation
data, builds a directed graph honoring airway directionality and high/low levels,
and finds routes between two airports (or waypoints) with A* and Yen K-shortest.
Airports connect to the enroute network through their real SID/STAR procedures
(parsed from ARINC 424 / CIFP), falling back to a direct link where no procedure
data exists. For example, `KJFK KLAX` resolves to a filed-flight-plan-style route
such as `KJFK DEEZZ5 TOWIN ... PGS BASET5 KLAX` of ~2160 NM.

A single loaded database is safe to query concurrently from multiple threads.

### Roadmap

- **M1 (done)** — enroute airway network, A* search, `bf route` CLI.
- **M2 (done)** — pluggable constraints: altitude bands, MORA safety floor,
  high/low level preference; Yen K-shortest for multiple candidate routes.
- **M3 (done)** — SID/STAR/approach procedures (ARINC 424 / CIFP, all 23 path
  terminators), terminal-area MSA, procedure-based airport connection, procedures
  surfaced in the route and CLI output.
- **M4 (done)** — a procedure exposes every on-network fix it passes as a
  candidate connection (with an along-track seed); the K-shortest search spans
  different connection fixes / procedures rather than one fixed pair;
  radar-vectored departures are flagged distinctly instead of looking like
  missing data; and `bf build` writes a compact, portable `.bfdb` cache that
  `bf route --db` loads for instant startup (~1.5s -> ~50ms). A full-corpus
  review showed the originally-planned "equivalent modeling of heading/arc/
  altitude legs" is not needed for routing (procedures already attach via
  definite fixes; the rest is distance-less and belongs to the later geometry
  work).

Not planned for the first phase: Web API, map visualization.

## Building

Requires a C++20 compiler and CMake (3.21+). Dependencies (Catch2, CLI11, RapidJSON)
are fetched automatically via FetchContent.

```bash
cmake --preset debug              # or: release
cmake --build --preset debug      # parallel build (use --preset, not the path form)
ctest --preset debug              # unit tests run always; integration tests need data
```

A `tsan` preset (ThreadSanitizer) is available to verify concurrency safety:

```bash
cmake --preset tsan && cmake --build --preset tsan && ctest --preset tsan
```

## Usage

```bash
# Build a binary cache once per AIRAC cycle for fast startup (~1.5s -> ~50ms)
bf build navdata                    # writes navdata/nav.bfdb
bf build /path/to/xplane -o my.bfdb

# Find a route (reads navigation data from ./navdata by default)
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KSEA KBOS --data /path/to/xplane/data

# Load the prebuilt cache to skip parsing (--data still locates CIFP files
# for on-demand procedure parsing)
bf route KJFK KLAX --db navdata/nav.bfdb

# Constrain by cruise altitude (enables altitude-band and MORA filtering)
bf route KJFK KLAX --alt 350

# Prefer high (Jet) or low (Victor) airways; ask for several candidates
bf route KJFK KLAX --level high -k 3

# Restrict the departure/arrival runway used for SID/STAR selection
bf route KJFK KLAX --rwy-dep RW31L
```

Endpoints are airport ICAO codes or waypoint idents, case-insensitive. When an
airport has procedure data, the route and its legs name the SID and STAR used (and
the interchangeable procedures that share the same connection fix).

The `.bfdb` cache is a portable, little-endian binary snapshot of the built
graph. It is derived from Navigraph/Jeppesen data and, like the source data, must
not be redistributed (it is git-ignored).

## Navigation Data

Navigation data is **not** included and must be supplied by the user. It is
copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only,
and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## License

[MIT](LICENSE). Third-party dependencies: see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
Contributing conventions: see [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
