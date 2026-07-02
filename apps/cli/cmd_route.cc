#include "commands.h"

#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cli_common.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace bf::cli {

void RegisterRoute(CLI::App& app, int& exit_code) {
  struct Args {
    std::string departure;
    std::string arrival;
    std::string data_dir = "navdata";
    std::string db_path;
    std::string cifp_db_path;
    std::string cifp_load = "on-demand";
    std::string format = "text";
    std::string level = "none";
    std::string alt_spec;
    std::string rwy_dep;
    std::string rwy_arr;
    std::string sid;
    std::string star;
    std::vector<std::string> avoid_wpt;
    std::vector<std::string> avoid_awy;
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
                    "Prebuilt .bfdb cache to load (skips parsing; --data still "
                    "locates CIFP files for procedures)");
  route->add_option("--cifp-db", a->cifp_db_path,
                    "CIFP procedure cache to load (default: <db-stem>_cifp.bfdb "
                    "next to --db, if present)");
  route->add_option("--cifp-load", a->cifp_load,
                    "CIFP cache load mode: on-demand (default) or eager")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
  route->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));
  route->add_option("--alt", a->alt_spec,
                    "Cruise flight level or range, e.g. 350 or 300-400 (enables "
                    "altitude/MORA filters)");
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
  route->add_option("--avoid-wpt", a->avoid_wpt,
                    "Waypoint(s) to avoid; ident or IDENT/REGION. Repeatable.");
  route->add_option("--avoid-awy", a->avoid_awy,
                    "Airway designator(s) to avoid, e.g. J60. Repeatable.");
  route->add_option("--seed", a->seed,
                    "Randomize routing with this seed for reproducible route diversity");
  route->add_option("--via", a->via,
                    "Force the route through these waypoint(s), in order; ident or "
                    "IDENT/REGION. Repeatable.");

  route->callback([a, &exit_code]() {
    // With --db, load the prebuilt cache (milliseconds); otherwise parse and
    // build from the data directory. --data still locates CIFP files for
    // on-demand procedure parsing; --cifp-db (or a sibling <stem>_cifp.bfdb)
    // supplies procedures from a cache instead.
    Result<NavDatabase> db =
        OpenForRead(a->db_path, a->data_dir, a->cifp_db_path, a->cifp_load);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }

    RouteRequest request;
    request.departure = a->departure;
    request.arrival = a->arrival;
    if (!a->alt_spec.empty()) {
      std::optional<FlRange> range = ParseAltSpec(a->alt_spec);
      if (!range) {
        std::cerr << "error: invalid --alt '" << a->alt_spec
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
    request.avoid_airways = a->avoid_awy;
    request.random_seed = a->seed;
    request.forced_points = a->via;
    if (a->level == "low") {
      request.level = LevelPreference::kLow;
    } else if (a->level == "high") {
      request.level = LevelPreference::kHigh;
    }

    Result<std::vector<Route>> result = db.value().FindRoutes(request);
    if (!result) {
      std::cerr << "error: " << result.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }

    const std::vector<Route>& routes = result.value();
    if (a->format == "json") {
      PrintRoutesJson(routes);
    } else {
      for (size_t i = 0; i < routes.size(); ++i) {
        if (routes.size() > 1) {
          std::cout << "=== Route " << (i + 1) << " of " << routes.size() << " ===\n";
        }
        PrintText(routes[i]);
        if (i + 1 < routes.size()) {
          std::cout << "\n";
        }
      }
    }
  });
}

}  // namespace bf::cli
