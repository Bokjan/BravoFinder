// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/routing/route_request.h"
#include "io/nav_database.h"
#include "render.h"

namespace bf::cli {

// Open a database for a read-only command (route / query / parse-route). With a
// non-empty `db_path`, loads the prebuilt unified .bfdb cache (graph + CIFP +
// detail in one file); otherwise parses the raw data under `data_dir`.
// `cifp_load` is "on-demand" or "eager" and only applies to the cached path.
// Returns the ready database or an Error.
Result<NavDatabase> OpenForRead(const std::string& db_path, const std::string& data_dir,
                                const std::string& cifp_load = "on-demand");

// Parse an --alt spec into an inclusive flight-level range. Accepts a single
// level ("350" -> {350, 350}) or a hyphenated range ("300-400" -> {300, 400}).
// Returns nullopt on malformed input, an inverted range (min > max), or a level
// outside 0..bf::service::kMaxFl (inclusive).
std::optional<FlRange> ParseAltSpec(const std::string& spec);

// Parse one --airway-filter spec into a structured AirwayRule. The syntax is
//
//   <region>[,<region>...]:<designator>[,<designator>...][=<action>[:<fraction>]]
//
// where a trailing '*' marks a prefix match and a bare '*' on either side means
// "everything":
//
//   ZB,ZG,ZH,ZJ,ZL,ZP,ZS,ZU,ZW,ZY:J*=block   all J routes in mainland China
//   Z*:J*=block                              same, but also ZM Mongolia / ZK North Korea
//   VI,VA,VO,VE:J*=penalize:0.3              soft penalty in Indian airspace
//   *:V*=penalize:0.8                         every V airway, worldwide
//   *:J60=block                              exactly J60 (no '*' => exact match)
//   ZB,ZG:J60,A3=block                       exactly J60 and A3, only in ZB/ZG
//   Z*:J*                                    defaults to penalize with fraction 0.5
//
// Regions are always prefix-matched (a code is at most two characters, so a
// two-character entry is already exact) and therefore take no mode marker beyond
// the optional cosmetic '*'. Designators need the distinction: 1371 designators in
// AIRAC 2601 are a strict prefix of another one, so a prefix "J60" would also match
// J603/J604/J605. All designators in ONE rule must agree on the marker, since the
// match mode is per rule; mixing them ("J*,A3") is rejected so the user splits the
// intent into two --airway-filter values rather than getting a silent guess.
//
// On failure, returns nullopt and sets `error` to a message naming what was wrong.
std::optional<AirwayRule> ParseAirwayFilter(const std::string& spec, std::string& error);

// Wrap a rendered routes array (the bare transport-shape body the query layer
// returns for find_routes) in the CLI's {"routes": <body>, "elapsed_ms": <n>}
// envelope, built with a RapidJSON Writer + RawValue so the outer object is not
// hand-rolled.
std::string WrapRoutesEnvelope(const std::string& routes_body, uint32_t elapsed_ms);

// Wrap a single rendered route object (parse_route) in the CLI's
// {"route": <body>, "elapsed_ms": <n>} envelope -- the singular "route" key
// matches parse_route's single-object result (vs the "routes" array of
// find_routes).
std::string WrapRouteEnvelope(const std::string& route_body, uint32_t elapsed_ms);

}  // namespace bf::cli
