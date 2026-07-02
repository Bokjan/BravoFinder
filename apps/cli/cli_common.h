#pragma once

#include <optional>
#include <string>
#include <vector>

#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace bf::cli {

// Open a database for a read-only command (route / query / parse-route). With a
// non-empty `db_path`, loads the prebuilt .bfdb cache (and a sibling / explicit
// CIFP cache); otherwise parses the raw data under `data_dir`. `cifp_load` is
// "on-demand" or "eager" and only applies to the cached path. Returns the ready
// database or an Error.
Result<NavDatabase> OpenForRead(const std::string& db_path, const std::string& data_dir,
                                const std::string& cifp_db_path,
                                const std::string& cifp_load = "on-demand");

// Print a route in human-readable text form (route string, distance, terminal
// procedures, forced points, and the per-leg table).
void PrintText(const Route& route);

// Print candidate routes as a JSON array.
void PrintRoutesJson(const std::vector<Route>& routes);

// Parse an --alt spec into an inclusive flight-level range. Accepts a single
// level ("350" -> {350, 350}) or a hyphenated range ("300-400" -> {300, 400}).
// Returns nullopt on malformed input or an inverted range (min > max).
std::optional<FlRange> ParseAltSpec(const std::string& spec);

}  // namespace bf::cli
