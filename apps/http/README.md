# bf-http

BravoFinder's HTTP query server: it exposes the `bf route` and `bf query` capabilities as HTTP+JSON endpoints for an internal (e.g. Go) gateway to call over the network. This file is written for **client/integration authors**: it is the request/response contract for every endpoint. The design rationale and threading model live in [docs/http-service.zh-CN.md](../../docs/http-service.zh-CN.md).

## What it is

- Transport: **HTTP/1.1 + JSON** over a hand-rolled [libuv](https://github.com/libuv/libuv) (async I/O + worker threadpool) and [llhttp](https://github.com/nodejs/llhttp) (request parser) stack. One event-loop thread does all I/O; each route computation is offloaded to the threadpool, so one loop scales.
- Capabilities: the nine query endpoints below (mirroring the CLI `route` / `query` subcommands), plus `/v1/cycles` and the `/healthz` / `/readyz` probes.
- Data: pointed at a **directory** of prebuilt `.bfdb` caches; serves one or more AIRAC cycles from it. It never parses raw data or writes files.

## Build and run

```bash
cmake --preset release && cmake --build --preset release
# binary: build/release/apps/http/bf-http

# The directory is --db-dir, else BRAVOFINDER_NAVDATA, else navdata/.
bf-http --db-dir /path/to/caches --host 0.0.0.0 --port 8080
```

Flags: `--db-dir DIR`, `--host` (default `0.0.0.0`), `--port` (default `8080`), `--worker-threads N` (libuv threadpool size, default = hardware concurrency), `--max-body BYTES` (request body cap, default 1 MiB), `--io-timeout SEC` (header/body read + idle keep-alive timeout, default 30), and `--version` (prints the program version and exits). It fails fast (non-zero exit, reason on stderr) if the directory holds no usable `nav_<cycle>.bfdb`.

## Conventions

- **Method + body**: every query endpoint is `POST` with a JSON **object** body and `Content-Type: application/json`. Batch lookups take `{"ids":[...]}`; a single lookup is just a one-element array. Responses are JSON with `Content-Type: application/json` and `Content-Length` (never chunked). The server honors `Connection: keep-alive`.
- **Cycle selection**: query endpoints accept `?cycle=2601` to pick an AIRAC cycle; omit it for the newest loaded cycle. The value is matched literally (not URL-decoded), so percent-encoding it (`24%33001`) yields `400`; a plain integer that names no loaded cycle is also `400`.
- **Numbers**: route distances and point coordinates are emitted at 6 decimal places (coordinates are ~0.1 m; the distance fields simply carry a few extra digits). Lookup coordinates/frequencies are also 6 dp.
- **Errors**: any non-2xx response has body `{"error":"<message>"}`. See [Error model](#error-model).
- **Binding**: `--host` expects an IPv4 address (the server binds via IPv4 only).

## Endpoint reference

| Method | Path | Purpose | Success |
|---|---|---|---|
| POST | `/v1/routes` | find k candidate routes | 200 |
| POST | `/v1/parse-route` | validate/expand a filed route string | 200 |
| POST | `/v1/waypoints` | batch waypoint/navaid lookup (grouped) | 200 |
| POST | `/v1/airports` | batch airport lookup | 200 |
| POST | `/v1/procedures` | batch terminal-procedure summary | 200 |
| POST | `/v1/procedure-legs` | one named procedure's per-leg detail | 200 |
| POST | `/v1/airways` | batch airway lookup | 200 |
| POST | `/v1/navaid-detail` | batch navaid detail (grouped) | 200 |
| POST | `/v1/holds` | batch holding-pattern lookup (grouped) | 200 |
| GET | `/v1/cycles` | list servable AIRAC cycles | 200 |
| GET | `/healthz` | liveness (process is up) | 200 |
| GET | `/readyz` | readiness (newest cycle opens) | 200 / 503 |

### POST `/v1/routes`

Request body (`departure` and `arrival` required, the rest optional):

```json
{
  "departure": "KJFK", "arrival": "KLAX",
  "min_fl": 300, "max_fl": 400,
  "level": "none",
  "k": 3,
  "departure_runway": "RW31L", "arrival_runway": "RW25L",
  "departure_sid": "DEEZZ5", "arrival_star": "LENDY6",
  "avoid_waypoints": ["BOTON", "PSB/K6"],
  "avoid_airways": ["J60"],
  "random_seed": 42,
  "forced_points": ["PSB"]
}
```

- `departure` / `arrival`: airport ICAO or waypoint ident (case-insensitive).
- `min_fl` / `max_fl`: inclusive cruise flight-level range (hundreds of feet); give only one for a single level. Setting either enables altitude-band / MORA filtering.
- `level`: `none` (default) | `low` (prefer Victor low airways) | `high` (prefer Jet high airways).
- `k`: number of candidate routes (Yen K-shortest), default 1, must be ≥ 1.
- `departure_runway` / `arrival_runway`: restrict the SID / STAR to this runway (e.g. `RW31L`); empty = any.
- `departure_sid` / `arrival_star`: pin a SID / STAR by name (e.g. `DEEZZ5`, or `DEEZZ5.TOWIN` to pin the transition); empty = auto.
- `avoid_waypoints`: idents (`BOTON`) or `IDENT/ARINC424_ICAO_CODE` (`BOTON/LF`) to route around; a bare ident avoids all its regional matches.
- `avoid_airways`: designators (e.g. `J60`) to route around; also blocks concurrency segments recorded as `J60-V123`.
- `random_seed`: reproducible route-diversity seed; omit for the plain optimum.
- `forced_points`: ordered via points, idents or `IDENT/ARINC424_ICAO_CODE`; echoed resolved.

Response `200`: an **array** of route objects, each:

```json
{
  "route": "KJFK SID CANDR ... STAR KLAX",
  "total_distance_nm": 2190.26,
  "dep_distance_nm": 77.43,
  "enroute_distance_nm": 2098.6,
  "arr_distance_nm": 14.22,
  "sid": "DEEZZ5", "dep_runway": "", "sid_options": ["DEEZZ5.CANDR", "..."],
  "star": "BASET5", "arr_runway": "", "star_options": ["BASET5", "..."],
  "forced_points": ["PSB/K6"],
  "dep_connection": "procedure", "arr_connection": "procedure",
  "points": [{"ident": "KJFK", "lat": 40.970986, "lon": -74.959827}, ...],
  "legs": [
    {"from": "KJFK", "to": "CANDR", "via": "SID", "distance_nm": 77.43, "cumulative_nm": 77.43},
    {"from": "SPOTZ", "to": "MIKYG", "via": "Q480", "concurrent_airways": ["Q42", "Q480"], "distance_nm": 22.17, "cumulative_nm": 122.44}
  ]
}
```

- `route` / `legs[].via`: the airport↔network procedure legs use the literal connector keyword `SID`/`STAR` (or `DCT` for a direct link); the procedure names themselves are in the `sid` / `star` fields.
- `dep_connection` / `arr_connection`: `procedure` | `direct` | `radar_vectors`.
- `points` has N entries, `legs` has N−1; the destination of `legs[i]` is `points[i+1]`, and the last `cumulative_nm` equals `total_distance_nm`.
- `forced_points` is present only when via points were given; `concurrent_airways` is present only on a concurrency leg.

Status: missing `departure`/`arrival`, `min_fl > max_fl`, or `k < 1` → **400**; an unknown endpoint or no feasible route → **422**.

### POST `/v1/parse-route`

Request: `{"route": "KJFK SID CANDR J60 PSB ... STAR KLAX"}` (required). The string is `DEP (SID|DCT) FIX (AWY FIX | DCT FIX)* (STAR|DCT) ARR`: both endpoints are airport ICAO codes and every adjacent waypoint pair carries an explicit connector (`DCT`, an airway, or `SID`/`STAR`). The no-fix form `DEP DCT ARR` is also accepted. Each airway must connect its bracketing fixes (expanded to intermediate points), else the error names the offending token. The `SID`/`STAR` slots accept either the literal keyword or a published procedure name (e.g. `DEEZZ5`); the returned `route` canonicalizes both to the keyword. Response `200`: a **single** route object, same shape as above. Status: missing `route` → **400**; a parse failure → **422**.

### Batch lookups

Request (all four non-grouped and grouped lookups): `{"ids": ["...", "..."]}`. The response is an array **parallel to `ids`**.

| Endpoint | Element on hit | On miss |
|---|---|---|
| `/v1/airports` | `{icao, arinc424_icao_code, lat, lon, elevation_ft, has_procedures}` | `null` |
| `/v1/procedures` | `{icao, procedures:[{type, name, transition, runway}]}` | `null` |
| `/v1/airways` | `{name, segments:[{from, to, distance_nm, high, base_fl, top_fl}]}` | `null` |
| `/v1/waypoints` | `WaypointInfo[]` — `{ident, arinc424_icao_code, lat, lon, kind, on_network}` | `[]` |
| `/v1/navaid-detail` | `NavaidDetailInfo[]` — `{ident, arinc424_icao_code, kind, elev_ft, freq_raw, range_nm, heading}` | `[]` |
| `/v1/holds` | `HoldInfo[]` — `{fix_ident, fix_arinc424_icao_code, airport_icao, inbound_course, leg_time_min, leg_dist_nm, turn_dir, min_alt_ft, max_alt_ft, speed_limit_kt}` | `[]` |

- The grouped lookups (`waypoints` / `navaid-detail` / `holds`) return an inner **array** per id (an ident recurs across regions), empty when unmatched.
- `freq_raw` is kHz for NDBs, MHz×100 for VOR/DME/ILS. `airport_icao` is `ENRT` for enroute holds.
- Status: missing/non-string `ids` → **400**; **every** id missing → **404** (a partial hit is 200).

### POST `/v1/procedure-legs`

Request: `{"airport": "KJFK", "procedure": "DEEZZ5"}` (both required). Response `200`:

```json
{
  "icao": "KJFK", "procedure": "DEEZZ5",
  "transitions": [
    {"type": "sid", "name": "DEEZZ5", "transition": "RW04B", "runway": "RW04B",
     "legs": [
       {"fix": "DEEZZ", "path_term": "TF", "course_deg": 273.0, "distance_nm": 12.3,
        "alt": "+2000", "rnp_nm": 1.0, "turn_dir": "L", "speed_limit_kt": 250}
     ]}
  ]
}
```

Optional leg fields (`alt`, `rnp_nm`, `turn_dir`, `speed_limit_kt`) are omitted when absent (a leg like a `VA`/`VM` path terminator has no fix, so `fix` is `""`). `type` is lowercase (`sid` / `star` / `appch`). Status: missing `airport`/`procedure` → **400**; unknown airport, no CIFP data, or no such procedure → **404**.

### GET `/v1/cycles`

Response `200`: `{"cycles": [{"cycle": 2601}, ...]}`, newest first.

### GET `/healthz` / `/readyz`

- `/healthz` → always `200 {"status":"ok"}` (the process is up).
- `/readyz` → `200 {"status":"ready"}` if the newest cycle opens, else `503 {"error":"..."}`.

## Error model

Every non-2xx response body is `{"error":"<message>"}`.

| Status | Meaning |
|---|---|
| **400** | Bad request: malformed JSON, missing/invalid field (`k<1`, `min_fl>max_fl`), or a bad/unserved `?cycle=`. |
| **404** | Nothing matched: all lookup ids missing, no such procedure, or an unregistered path. |
| **422** | Well-formed but unsatisfiable: unknown endpoint, no feasible route, a bad route token. |
| **413** | Request body exceeds `--max-body`. |
| **400** | `Transfer-Encoding: chunked` request body (refused; the server only accepts `Content-Length`). |
| **500** | Unexpected server error. |
| **503** | `/readyz` only: the newest cycle cannot be opened. |

The distinction that matters: **400** = "you sent it wrong"; **422** = "you sent it right, but there is no answer".

## Examples

```bash
# Find one route, pretty-printed.
curl -s -X POST localhost:8080/v1/routes \
  -H 'Content-Type: application/json' \
  -d '{"departure":"KJFK","arrival":"KLAX","k":1}' | jq .

# Batch airport lookup (one hit, one miss -> [obj, null]).
curl -s -X POST localhost:8080/v1/airports -d '{"ids":["KJFK","ZZZZ"]}'

# Pick a specific AIRAC cycle.
curl -s -X POST 'localhost:8080/v1/routes?cycle=2601' \
  -d '{"departure":"EGLL","arrival":"LFPG"}'

# Probes and cycle list.
curl -s localhost:8080/healthz
curl -s localhost:8080/readyz
curl -s localhost:8080/v1/cycles
```

## Concurrency and lifetime

Each `NavDatabase` is read-only after open and safe for concurrent queries (thread-safety contract); the registry that opens cycles on demand is thread-safe. The server holds one long-lived instance and serves many concurrent connections — no restart is needed to switch cycles. Route computations run on the worker threadpool, so a slow query never blocks other connections.
