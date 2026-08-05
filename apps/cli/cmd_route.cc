// SPDX-License-Identifier: MIT
#include <cstdint>
#include <cstdlib>
#include <format>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cli_common.h"
#include "commands.h"
#include "core/constraints/airway_rule_constraint.h"
#include "core/routing/route_request.h"
#include "handlers.h"
#include "queries.h"
#include "render.h"

namespace bf::cli {

void RegisterRoute(CLI::App& app, int& exit_code) {
  struct Args {
    std::string departure;
    std::string arrival;
    std::string data_dir = "navdata";
    std::string db_path;
    std::string cifp_load = "on-demand";
    std::string format = "text";
    std::string level = "none";
    std::string alt_spec;
    std::string rwy_dep;
    std::string rwy_arr;
    std::string sid;
    std::string star;
    std::vector<std::string> avoid_wpt;
    std::vector<std::string> airway_filter;
    std::optional<uint32_t> seed;
    std::vector<std::string> via;
    int k = 1;
  };
  auto a = std::make_shared<Args>();

  CLI::App* route = app.add_subcommand("route", "Find a route between two points");
  route->add_option("departure", a->departure, "Departure ICAO or waypoint ident")->required();
  route->add_option("arrival", a->arrival, "Arrival ICAO or waypoint ident")->required();
  route->add_option("--data", a->data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  route->add_option("--db", a->db_path,
                    "Prebuilt unified .bfdb cache to load (skips parsing; graph, "
                    "CIFP procedures, and detail all live in this one file)");
  route
      ->add_option("--cifp-load", a->cifp_load,
                   "CIFP cache load mode: on-demand (default) or eager")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
  route->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));
  route->add_option("--alt", a->alt_spec,
                    std::format("Cruise flight level or range, e.g. 350 or 300-400 (enables "
                                "altitude/MORA filters; accepted FL is 0..{})",
                                bf::service::kMaxFl));
  route->add_option("--level", a->level, "Airway level preference: none, low, high")
      ->capture_default_str()
      ->check(CLI::IsMember({"none", "low", "high"}));
  route->add_option("-k", a->k, "Number of candidate routes to return")
      ->capture_default_str()
      ->check(CLI::PositiveNumber);
  route->add_option("--rwy-dep", a->rwy_dep,
                    "Departure runway to restrict the SID, e.g. RW31L (default: any)");
  route->add_option("--rwy-arr", a->rwy_arr,
                    "Arrival runway to restrict the STAR, e.g. RW25L (default: any)");
  route->add_option("--sid", a->sid,
                    "Select a specific SID by name, e.g. DEEZZ5 or DEEZZ5.TOWIN (default: auto)");
  route->add_option("--star", a->star,
                    "Select a specific STAR by name, e.g. LENDY6 or LENDY6.HAAYS (default: auto)");
  // The vector options take exactly one value per occurrence and are repeated for
  // more. Without allow_extra_args(false), CLI11 lets a vector option greedily
  // absorb every following argument, so it swallows the positional
  // departure/arrival and "--avoid-wpt BOTON KJFK KLAX" failed with "departure is
  // required". Note this is NOT expected(1), which would instead cap the option at
  // a single occurrence and break the repeatable form.
  route
      ->add_option("--avoid-wpt", a->avoid_wpt,
                   "Waypoint(s) to avoid; ident or IDENT/ARINC424_ICAO_CODE. Repeatable.")
      ->allow_extra_args(false);
  route
      ->add_option(
          "--airway-filter", a->airway_filter,
          std::format(
              "Restrict airways by ICAO region and designator: "
              "<regions>:<designators>[=block|penalize[:<fraction>]]. A trailing '*' means "
              "prefix, no '*' means exact, a bare '*' means any; comma-separate to list "
              "several. Defaults to penalize with fraction {}. Repeatable (each value is one "
              "rule, max {}). E.g. ZB,ZG,ZH,ZJ,ZL,ZP,ZS,ZU,ZW,ZY:J*=block (all J routes in "
              "mainland China), *:J60=block (exactly J60, anywhere), *:V*=penalize:0.8.",
              bf::kDefaultPenaltyFraction, bf::AirwayRuleConstraint::kMaxRules))
      ->allow_extra_args(false);
  route->add_option("--seed", a->seed,
                    "Randomize routing with this seed for reproducible route diversity");
  route
      ->add_option("--via", a->via,
                   "Force the route through these waypoint(s), in order; ident or "
                   "IDENT/ARINC424_ICAO_CODE. Repeatable.")
      ->allow_extra_args(false);

  route->callback([a, &exit_code]() {
    // With --db, load the prebuilt cache (milliseconds); otherwise parse and
    // build from the data directory. On the cached path, procedures come from
    // the same unified .bfdb; on the raw path, --data locates CIFP files for
    // on-demand procedure parsing.
    Result<NavDatabase> db = OpenForRead(a->db_path, a->data_dir, a->cifp_load);
    if (!db) {
      std::cerr << bf::service::kTextErrorPrefix << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }

    RouteRequest request;
    request.departure = a->departure;
    request.arrival = a->arrival;
    if (!a->alt_spec.empty()) {
      std::optional<FlRange> range = ParseAltSpec(a->alt_spec);
      if (!range) {
        std::cerr << bf::service::kTextErrorPrefix << "invalid --alt '" << a->alt_spec
                  << "' (expected a level like 350 or a range like 300-400)\n";
        exit_code = EXIT_FAILURE;
        return;
      }
      request.altitude = range;
    }
    request.k = a->k;
    request.departure_runway = a->rwy_dep;
    request.arrival_runway = a->rwy_arr;
    request.departure_sid = a->sid;
    request.arrival_star = a->star;
    request.avoid_waypoints = a->avoid_wpt;
    for (const std::string& spec : a->airway_filter) {
      std::string error;
      std::optional<AirwayRule> rule = bf::cli::ParseAirwayFilter(spec, error);
      if (!rule) {
        std::cerr << bf::service::kTextErrorPrefix << "invalid --airway-filter '" << spec
                  << "': " << error << "\n";
        exit_code = EXIT_FAILURE;
        return;
      }
      request.airway_rules.push_back(std::move(*rule));
    }
    request.random_seed = a->seed;
    request.forced_points = a->via;
    if (a->level == "low") {
      request.level = LevelPreference::kLow;
    } else if (a->level == "high") {
      request.level = LevelPreference::kHigh;
    }

    // Delegate to the shared query layer: it runs FindRoutes, renders in the
    // requested format, and returns a body + HTTP-style status (+ elapsed_ms,
    // which the text renderer folds into the body). The CLI is a thin front-end
    // over the same path the MCP / HTTP transports use.
    const bf::service::OutputFormat fmt =
        a->format == "json" ? bf::service::OutputFormat::kJson : bf::service::OutputFormat::kText;
    const bf::service::HandlerResult result = bf::service::FindRoutes(db.value(), request, fmt);
    if (result.status >= bf::service::kErrorStatusThreshold) {
      std::cerr << result.body;
      if (fmt == bf::service::OutputFormat::kJson) {
        std::cerr << "\n";
      }
      exit_code = EXIT_FAILURE;
      return;
    }
    if (fmt == bf::service::OutputFormat::kJson) {
      // The query layer renders a bare routes array (the transport shape); the
      // CLI wraps it with the elapsed_ms envelope it has shipped since v3.13.0.
      std::cout << bf::cli::WrapRoutesEnvelope(result.body, result.elapsed_ms) << "\n";
    } else {
      std::cout << result.body;
    }
  });
}

}  // namespace bf::cli
