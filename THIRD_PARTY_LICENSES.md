# Third-Party Licenses

BravoFinder v3 is licensed under the MIT License (see `LICENSE`).

It depends on the following third-party libraries, fetched at build time via CMake
FetchContent. Their source is **not** vendored into this repository. All of them use
permissive licenses compatible with MIT.

| Library | Purpose | License | License URL |
|---|---|---|---|
| [Catch2](https://github.com/catchorg/Catch2) v3 | Unit testing framework | Boost Software License 1.0 | https://www.boost.org/LICENSE_1_0.txt |
| [CLI11](https://github.com/CLIUtils/CLI11) | Command-line parsing | BSD-3-Clause | https://github.com/CLIUtils/CLI11/blob/main/LICENSE |
| [RapidJSON](https://github.com/Tencent/rapidjson) | JSON output (`route --format json`) | MIT | https://github.com/Tencent/rapidjson/blob/master/license.txt |

## Obligations

- **Boost Software License 1.0** (Catch2): very permissive; no obligation to reproduce
  the notice in binary distributions.
- **BSD-3-Clause** (CLI11): requires the copyright notice and license text to be
  retained. This file, together with the upstream license bundled by FetchContent,
  satisfies that obligation.
- **MIT** (RapidJSON): requires the copyright notice and permission notice to be
  retained. RapidJSON also bundles a copy of the MIT-licensed msinttypes; both notices
  ship with the headers fetched by FetchContent. (Only the header-only Writer is used,
  for JSON serialization; the parser is not used.)

## Navigation Data (not a code dependency)

Navigation data (Navigraph / Jeppesen, X-Plane format) used at runtime is **not**
part of this project and is **not** redistributed here. It is copyrighted, licensed
for recreational simulation use only, and must not be redistributed. Users supply
their own data locally (see `navdata/`).
