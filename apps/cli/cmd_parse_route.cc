#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "cli_common.h"
#include "commands.h"
#include "core/routing/route.h"
#include "io/nav_database.h"

namespace bf::cli {

void RegisterParseRoute(CLI::App& app, int& exit_code) {
  struct Args {
    std::string route_str;
    std::string data_dir = "navdata";
    std::string db_path;
    std::string cifp_db_path;
    std::string format = "text";
  };
  auto a = std::make_shared<Args>();

  CLI::App* parse = app.add_subcommand(
      "parse-route", "Validate and expand a filed route string (reverse of route)");
  parse
      ->add_option("route", a->route_str,
                   "Filed route string, e.g. \"KJFK DEEZZ5 CANDR J60 PSB ... KLAX\"")
      ->required();
  parse->add_option("--data", a->data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  parse->add_option("--db", a->db_path, "Prebuilt .bfdb cache to load (skips parsing)");
  parse->add_option("--cifp-db", a->cifp_db_path,
                    "CIFP procedure cache to load (default: <db-stem>_cifp.bfdb next to --db)");
  parse->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  parse->callback([a, &exit_code]() {
    Result<NavDatabase> db = OpenForRead(a->db_path, a->data_dir, a->cifp_db_path);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    Result<Route> result = db.value().ParseRoute(a->route_str);
    if (!result) {
      std::cerr << "error: " << result.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    if (a->format == "json") {
      PrintRoutesJson({result.value()});
    } else {
      PrintText(result.value());
    }
  });
}

}  // namespace bf::cli
