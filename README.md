# BravoFinder

A flight route finder written in modern C++, **version 3.0.0 — a complete rewrite**.

## About

BravoFinder builds a graph from navigation data (waypoints, navaids, airways, and
SID/STAR/approach procedures) and finds routes between two airports. Unlike earlier
versions, which computed a purely geographic shortest path, v3 is a **realistic /
compliant route engine**: routes respect real-world constraints such as airway
directionality, high/low airway levels, segment altitude bands, and terminal
procedures.

## Status

v3.0.0 is the first release. Everything below is implemented and tested; the CLI
(`bf build` / `bf route`) is usable end to end against real X-Plane 12 data.

The tool loads X-Plane 12 navigation data, builds a directed graph honoring airway
directionality and high/low levels, and finds routes between two airports (or
waypoints) with A* and Yen K-shortest. Airports connect to the enroute network
through their real SID/STAR procedures (parsed from ARINC 424 / CIFP), falling back
to a direct link where no procedure data exists. For example, `KJFK KLAX` resolves
to a filed-flight-plan-style route such as `KJFK DEEZZ5 TOWIN ... PGS BASET5 KLAX`
of ~2160 NM.

A single loaded database is safe to query concurrently from multiple threads.

### What each milestone delivered

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

### MCP server (`bf-mcp-stdio`)

BravoFinder also ships a local MCP server that exposes `bf route` and `bf query`
as MCP tools over stdio, so an LLM client can ask for routes and look up
navigation data directly. It is a thin, zero-dependency (beyond the project's
own library) stdio JSON-RPC server: it loads the `.bfdb` cache once at startup
and serves queries from it.

Build it alongside the CLI:

```bash
cmake --preset release && cmake --build --preset release   # or: debug
# binary: build/release/apps/mcp_stdio/bf-mcp-stdio
```

Point it at a cache and run it (it fails fast at startup if the cache is
missing or corrupt):

```bash
# Defaults: BRAVOFINDER_NAVDATA locates navdata/, which must contain nav.bfdb
# (and optionally nav_cifp.bfdb). Command-line flags override the environment.
BRAVOFINDER_NAVDATA=navdata bf-mcp-stdio
bf-mcp-stdio --db navdata/nav.bfdb --cifp-db navdata/nav_cifp.bfdb
```

Tools exposed (each maps 1:1 to a CLI subcommand):

| Tool | Description |
|------|-------------|
| `find_routes` | Route between two endpoints; same options as `bf route` (`level`, `k`, `cruise_fl`, runways, SID/STAR). |
| `lookup_waypoints` | Batch-look-up waypoints by ident. |
| `lookup_airports` | Batch-look-up airports by ICAO. |
| `lookup_procedures` | Batch-look-up SID/STAR/approach by airport ICAO. |
| `lookup_airways` | Batch-look-up airways by designator. |

Example MCP client configuration (Claude Desktop / similar):

```json
{
  "mcpServers": {
    "bravofinder": {
      "command": "/abs/path/to/bf-mcp-stdio",
      "args": ["--db", "navdata/nav.bfdb"]
    }
  }
}
```

The server speaks MCP over stdio as JSON-RPC 2.0. It only reads the cache
(`NavDatabase` is immutable after `OpenCached`), so it is safe for an MCP client
to hold one long-lived instance. `bf build` (cache creation) remains a CLI
concern and is not exposed as a tool.

```bash
# Build binary caches once per AIRAC cycle for fast startup (~1.5s -> ~50ms).
# By default this writes both nav.bfdb (graph) and nav_cifp.bfdb (procedures),
# so deployment needs only the two cache files, not the CIFP/ directory.
bf build navdata                    # writes navdata/nav.bfdb + navdata/nav_cifp.bfdb
bf build /path/to/xplane -o my.bfdb # writes my.bfdb + my_cifp.bfdb
bf build navdata --without-cifp     # graph cache only

# Find a route (reads navigation data from ./navdata by default)
bf route KJFK KLAX
bf route EGLL LFPG --format json
bf route KSEA KBOS --data /path/to/xplane/data

# Load the prebuilt caches to skip parsing. The sibling nav_cifp.bfdb is
# auto-discovered next to --db; --cifp-db overrides it. With both caches, the
# CIFP/ directory is not needed at all.
bf route KJFK KLAX --db navdata/nav.bfdb
bf route KJFK KLAX --db navdata/nav.bfdb --cifp-db other_cifp.bfdb

# Procedure cache load mode: on-demand (default, ~1.5 MB, best for one-shot
# queries) or eager (loads all procedures up front, ~100 MB then lock-free,
# best for servers / batch routing)
bf route KJFK KLAX --db navdata/nav.bfdb --cifp-load eager

# Constrain by cruise altitude (enables altitude-band and MORA filtering)
bf route KJFK KLAX --alt 350

# Prefer high (Jet) or low (Victor) airways; ask for several candidates
bf route KJFK KLAX --level high -k 3

# Restrict the departure/arrival runway used for SID/STAR selection
bf route KJFK KLAX --rwy-dep RW31L

# Select a specific SID/STAR by name (bare name matches any transition;
# NAME.TRANSITION pins the transition). An unknown name is a clean error.
bf route KJFK KLAX --sid DEEZZ5
bf route KJFK KLAX --star LENDY6.HAAYS

# Look up navigation data: waypoints, airports, procedures, or airways. Each
# accepts one or more ids (a batch), and --format json emits an array parallel
# to the input (a not-found id becomes null).
bf query waypoint --db navdata/nav.bfdb NINOX DGC
bf query airport  --db navdata/nav.bfdb KJFK KLAX
bf query procedure --db navdata/nav.bfdb KJFK
bf query airway   --db navdata/nav.bfdb Y28 --format json

# Print the program version
bf --version
```

Endpoints are airport ICAO codes or waypoint idents, case-insensitive. When an
airport has procedure data, the route and its legs name the SID and STAR used (and
the interchangeable procedures that share the same connection fix).

The `.bfdb` caches are portable, little-endian binary snapshots (`nav.bfdb` holds
the graph; `nav_cifp.bfdb` holds per-airport procedures, loaded on demand). They
are derived from Navigraph/Jeppesen data and, like the source data, must not be
redistributed (they are git-ignored).

## Navigation Data

Navigation data is **not** included and must be supplied by the user. It is
copyrighted (Navigraph / Jeppesen), licensed for recreational simulation use only,
and must not be redistributed. Place your local data under `navdata/` (git-ignored).

## Documentation

In-depth technical articles live under [docs/](docs/) (in Chinese). Start with
[the routing-algorithm primer](docs/routing-basics.zh-CN.md), then explore the
design highlights: [compliant routing](docs/compliant-routing.zh-CN.md),
[procedure modeling](docs/procedure-modeling.zh-CN.md), the
[Yen / Lawler optimization](docs/yen-lawler-optimization.zh-CN.md), the
[portable binary cache](docs/binary-cache.zh-CN.md),
[thread-safety contract B](docs/thread-safety.zh-CN.md), and
[performance](docs/performance.zh-CN.md). See [docs/README.md](docs/README.md) for
the full index.

## License

[MIT](LICENSE). Third-party dependencies: see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).
Contributing conventions: see [docs/CONTRIBUTING.md](docs/CONTRIBUTING.md).
