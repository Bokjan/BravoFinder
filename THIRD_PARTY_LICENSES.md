# Third-Party Licenses

BravoFinder v3 is dual-licensed: files under `libs/engine/` use the GNU Lesser General Public License v3.0-or-later (see `libs/engine/LICENSE`), and all other files use the MIT License (see `LICENSE`).

It depends on the following third-party libraries, fetched at build time via CMake FetchContent. Their source is **not** vendored into this repository. All of them use permissive licenses compatible with the project's MIT and LGPL-3.0 licenses.

| Library | Purpose | License | License URL |
|---|---|---|---|
| [Catch2](https://github.com/catchorg/Catch2) v3 | Unit testing framework | Boost Software License 1.0 | https://www.boost.org/LICENSE_1_0.txt |
| [CLI11](https://github.com/CLIUtils/CLI11) | Command-line parsing | BSD-3-Clause | https://github.com/CLIUtils/CLI11/blob/main/LICENSE |
| [RapidJSON](https://github.com/Tencent/rapidjson) | JSON output (`route --format json`) | MIT | https://github.com/Tencent/rapidjson/blob/master/license.txt |
| [SQLite](https://www.sqlite.org/) 3.46.0 | DFD SQLite loaders (v1/v2) | Public Domain | https://www.sqlite.org/copyright.html |
| [libuv](https://github.com/libuv/libuv) v1.49.2 | Async I/O event loop + threadpool (`bf-http`) | MIT | https://github.com/libuv/libuv/blob/v1.49.2/LICENSE |
| [llhttp](https://github.com/nodejs/llhttp) v9.2.1 | HTTP/1.1 request parser (`bf-http`) | MIT | https://github.com/nodejs/llhttp/blob/release/v9.2.1/LICENSE-MIT |

## Obligations

- **Boost Software License 1.0** (Catch2): very permissive; no obligation to reproduce the notice in binary distributions.
- **BSD-3-Clause** (CLI11): requires the copyright notice and license text to be retained. This file, together with the upstream license bundled by FetchContent, satisfies that obligation.
- **MIT** (RapidJSON): requires the copyright notice and permission notice to be retained. RapidJSON also bundles a copy of the MIT-licensed msinttypes; both notices ship with the headers fetched by FetchContent. (Only the header-only Writer is used, for JSON serialization; the parser is not used.)
- **Public Domain** (SQLite): no obligations whatsoever. SQLite is dedicated to the public domain and may be used in any project without restriction.
- **MIT** (libuv, llhttp): require the copyright notice and permission notice to be retained. Unlike RapidJSON these are compiled into the `bf-http` binary, so release artifacts that ship `bf-http` bundle this license text (see the release workflow). libuv's `LICENSE` additionally includes a few permissive sub-notices (e.g. `tree.h` under BSD-2-Clause); retaining the upstream file as-is satisfies them, with no new obligation.

## Navigation Data (not a code dependency)

Navigation data (Navigraph / Jeppesen, X-Plane format) used at runtime is **not** part of this project and is **not** redistributed here. It is copyrighted, licensed for recreational simulation use only, and must not be redistributed. Users supply their own data locally (see `navdata/`).
