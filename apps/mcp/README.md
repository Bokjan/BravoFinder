# bf-mcp

BravoFinder's MCP server: it exposes the `bf route` and `bf query` capabilities as MCP tools for LLM clients, over **stdio** (default) or **HTTP** (Streamable HTTP). This file is written for **agents / configuration assistants**: the first half is how to run the server and wire it into an MCP client; the second half is a tool reference.

## What it is

- Transport: **stdio** (default — a local process the client spawns and talks to over stdin/stdout) or **HTTP** (`--transport http` — Streamable HTTP, 2025-03-26, on a TCP port `/mcp`, for remote / multi-client access). Both share the same protocol core and capabilities; only the byte transport differs.
- Protocol: MCP over JSON-RPC 2.0, hand-rolled with no third-party MCP SDK. Protocol version is negotiated at `initialize` (`2024-11-05` or `2025-03-26`).
- Capabilities: the nine per-database tools below (mirroring the CLI `route` / `query` subcommands), plus a `list_cycles` tool.
- Data: the server is pointed at a **directory** of prebuilt `.bfdb` caches and serves one or more AIRAC cycles from it. It never parses raw data or writes files.

## Multiple AIRAC cycles

The server reads a directory of `nav_<cycle>.bfdb` caches (built by `bf build`; see the repo-root README). Each unified `.bfdb` carries the graph, CIFP procedures, and navaid detail in three sections; its header records the authoritative cycle, so the directory can hold several AIRACs at once.

- Each cycle's database is opened **lazily** on first use and then cached; a cold cycle costs one open, subsequent queries are served from memory.
- Every per-database tool takes an optional `cycle` argument. Omit it to use the **latest** (highest cycle) loaded cache.
- `list_cycles` returns the available cycles.
- A single-cycle deployment is just a directory holding one cache — nothing special to configure.

## Build

Depends on the project's own `bf` library, RapidJSON, and (for the HTTP transport) the shared `bf_http_server` core (libuv + llhttp) — pure CMake + FetchContent, no extra setup.

```bash
cmake --preset release && cmake --build --preset release
# or debug:
cmake --preset debug   && cmake --build --preset debug
```

Output path (CMake target `bf_mcp`, external name `bf-mcp`):

```
build/release/apps/mcp/bf-mcp
build/debug/apps/mcp/bf-mcp
```

## Startup and cache location

At startup the server scans a directory for `nav_<cycle>.bfdb` caches and builds a registry from their headers. Each unified `.bfdb` holds the graph, CIFP procedures, and navaid detail in one file — no companion files are needed. Databases are opened lazily per cycle via the registry. If the directory holds no usable cache the process exits non-zero and prints the reason to stderr (fail-fast, so the MCP client reports a clean startup failure). Files whose header is unreadable are reported to stderr and skipped, not silently ignored.

The directory is resolved in this order:

1. Command-line `--db-dir <dir>`.
2. The `BRAVOFINDER_NAVDATA` environment variable.
3. Default `navdata/`.

```bash
# Simplest: navdata/ must contain at least one nav_<cycle>.bfdb, located
# via BRAVOFINDER_NAVDATA.
BRAVOFINDER_NAVDATA=navdata bf-mcp

# Explicit directory holding one or more caches.
bf-mcp --db-dir /path/to/caches

# Serve MCP over HTTP instead of stdio (single endpoint POST/GET/DELETE /mcp).
# HTTP-mode flags mirror bf-http: --host, --port, --worker-threads, --max-body,
# --io-timeout (all ignored in the default stdio mode).
bf-mcp --transport http --db-dir /path/to/caches --host 0.0.0.0 --port 8080
```

The HTTP transport is stateless-with-id: `initialize` returns a random `Mcp-Session-Id` header that the server does not track (every request is independent). `POST /mcp` carries a JSON-RPC request (single or batch) and answers `application/json`, or a single `text/event-stream` SSE event when the client's `Accept` header opts in; a notification-only body returns `202`. `GET /mcp` opens an SSE stream (a placeholder — no server-push today). `DELETE /mcp` acknowledges a session teardown. See [docs/http-service.zh-CN.md](../../docs/http-service.zh-CN.md) for the transport design.

`bf-mcp` also honors `--version`, which prints the program version and exits before scanning any caches.

> Caches are produced by `bf build` (see the repo-root README). `bf-mcp` itself does **not** expose `build` — creating caches is a CLI / deployment concern.

## Wiring into an MCP client

In the client's MCP server config, point `command` at the compiled binary and pass startup arguments via `args`. Example (Claude Desktop / Cursor / similar):

```json
{
  "mcpServers": {
    "bravofinder": {
      "command": "/abs/path/to/bf-mcp",
      "args": ["--db-dir", "/path/to/caches"]
    }
  }
}
```

Using an environment variable instead (most clients support an `env` field):

```json
{
  "mcpServers": {
    "bravofinder": {
      "command": "/abs/path/to/bf-mcp",
      "env": { "BRAVOFINDER_NAVDATA": "/path/to/navdata" }
    }
  }
}
```

## Tool reference

All tool request parameters go in the MCP `tools/call` `arguments` object. Results are uniformly wrapped as `{ content: [{ type: "text", text: <json> }], isError: <bool> }`, where `text` is a JSON value (object or array) ready to parse.

Every per-database tool also accepts an optional **`cycle`** (integer, e.g. `2601`); omit it for the latest loaded cycle. It is injected by the server and selects which database answers the call, so it is not repeated in each tool's row below.

| Tool | Required | Optional | Returns |
|------|----------|----------|---------|
| `find_routes` | `departure`, `arrival` | `min_fl`, `max_fl`, `level`, `k`, `departure_runway`, `arrival_runway`, `departure_sid`, `arrival_star`, `avoid_waypoints`, `avoid_airways`, `random_seed`, `forced_points` | Array of candidate routes (see below) |
| `parse_route` | `route` (string) | — | The validated/expanded single route object |
| `lookup_waypoints` | `ids` (string[]) | — | Array parallel to `ids`; each element is an array of region matches (empty if none) |
| `lookup_airports` | `ids` (string[]) | — | Array parallel to `ids`; airport object or `null` |
| `lookup_procedures` | `ids` (string[]) | — | Array parallel to `ids`; procedures object or `null` |
| `lookup_procedure_legs` | `airport`, `procedure` | — | The named procedure's transitions, each with its ordered legs (fix, path terminator, course, distance, altitude, RNP, turn, speed) |
| `lookup_airways` | `ids` (string[]) | — | Array parallel to `ids`; airway object or `null` |
| `lookup_navaid_detail` | `ids` (string[]) | — | Array parallel to `ids`; each element is an array of region matches (empty if none) |
| `lookup_holds` | `ids` (string[]) | — | Array parallel to `ids`; each element is an array of holds at that fix (empty if none) |
| `list_cycles` | — | — | Array of `{cycle}`, newest first |

### `find_routes`

Parameter semantics:

- `departure` / `arrival`: airport ICAO or waypoint ident (case-insensitive).
- `min_fl` / `max_fl`: inclusive cruise flight-level range (hundreds of feet), e.g. `min_fl=300, max_fl=400` for FL300–FL400; giving only one is a single level (e.g. only `min_fl=350` means FL350). Setting either enables altitude-band / MORA constraint filtering.
- `level`: `none` (default) | `low` (prefer Victor low airways) | `high` (prefer Jet high airways).
- `k`: number of candidate routes (Yen K-shortest), default 1, must be ≥ 1.
- `departure_runway` / `arrival_runway`: restrict the SID / STAR to this runway, e.g. `RW31L`; empty = any.
- `departure_sid` / `arrival_star`: pin a SID / STAR by name (e.g. `DEEZZ5`, or `DEEZZ5.TOWIN` to pin the transition); empty = auto.
- `avoid_waypoints`: waypoints to route around, each an ident (`BOTON`) or `IDENT/REGION` (`BOTON/LF`); a bare ident avoids all its regional matches.
- `avoid_airways`: airway designators to route around, e.g. `J60`; also blocks concurrency segments recorded as `J60-V123`.
- `random_seed`: reproducible route-diversity seed; the same seed always yields the same route, different seeds explore alternative valid routes; omit for the plain optimal route.
- `forced_points`: ordered via points, each an ident (`PSB`) or `IDENT/REGION` (`PSB/K6`); the response echoes them resolved as `IDENT/REGION`.

Each returned route element carries: `route` (ICAO filed-flight-plan string), `total_distance_nm`, `dep_distance_nm` / `enroute_distance_nm` / `arr_distance_nm`, `sid`, `dep_runway`, `sid_options`, `star`, `arr_runway`, `star_options`, `forced_points` (if any), `dep_connection` / `arr_connection` (`procedure` / `radar_vectors` etc.), `points` (each `{ident, lat, lon}`, ordered along the route), and `legs` (each with `from` / `to` / `via` / `distance_nm` / `cumulative_nm`; concurrency segments add `concurrent_airways`). `points` has one more entry than `legs` (the destination of leg *i* is `points[i+1]`). In `route` and in `legs[].via`, the airport↔network procedure legs show the literal connector keyword `SID`/`STAR` (or `DCT` for a direct link); the procedure names are in the `sid` / `star` fields.

### `parse_route`

`route` is an ICAO filed-flight-plan string
(`[DEP] [SID] FIX (AWY FIX | DCT FIX)* [STAR] [ARR]`). Each segment is validated:
an airway designator must actually connect its bracketing fixes (expanded to its intermediate points on the graph), else an error names the offending token. The `[SID]`/`[STAR]` slots accept either the literal keyword `SID`/`STAR` or a published procedure name (the latter checked against the endpoint airport's procedures); either way it becomes a single connection leg (not expanded leg-by-leg) and the returned `route` canonicalizes to the keyword. Returns a single route object with the same shape as `find_routes` (`route` / `total_distance_nm` / `legs` etc.).

### lookup tools

`ids` is a string array; the result array is **parallel to `ids`**, with `null` (or an empty inner array for the grouped lookups) where an id is not found. `lookup_procedures` takes airport ICAO codes; the others match by ident / designator.

`lookup_navaid_detail` and `lookup_holds` are grouped lookups (like `lookup_waypoints`): an ident maps to an array of matches. The data lives in the unified `.bfdb`'s detail section, built automatically by `bf build`. `lookup_navaid_detail` returns each navaid's `ident`, `region`, `kind`, `elev_ft`, `freq_raw` (kHz for NDBs, MHz×100 for VOR/DME/ILS), `range_nm`, and `heading`. `lookup_holds` returns each hold's `fix_ident`, `fix_region`, `airport_icao` (`ENRT` for enroute holds), `inbound_course`, `leg_time_min`, `leg_dist_nm`, `turn_dir`, `min_alt_ft`, `max_alt_ft`, and `speed_limit_kt`.

### `list_cycles`

Takes no arguments. Returns an array of `{cycle}` for the caches the server can serve, newest first. Pass a `cycle` to the other tools to query a specific one.

## Error semantics

- Successful tool call: `isError: false`.
- `find_routes` cannot compute a route (unknown endpoint / no feasible route): `isError: true`, `text` is `{"error":"..."}`.
- A lookup where **all** `ids` are missing: `isError: true`; a **partial** hit: `isError: false` (missing entries are `null` in the array).
- An unknown `cycle`: `isError: true`, `text` is `{"error":"unknown AIRAC cycle: ..."}`.
- Unknown tool name: `isError: true`.
- Protocol-level error (e.g. `tools/call` missing its `params` object): a JSON-RPC `error`, not a tool result.

## Concurrency and lifetime

Each `NavDatabase` is read-only after `OpenCached` and safe for concurrent queries (contract B). The registry that opens cycles on demand is likewise thread-safe: a mutex guards only the cache map, never the disk open, and opened databases have stable addresses. A client holds one long-lived server instance and calls `tools/call` repeatedly; no restart is needed to switch cycles.
