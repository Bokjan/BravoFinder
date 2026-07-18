#include <chrono>
#include <cstdint>
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
    std::string format = "text";
  };
  auto a = std::make_shared<Args>();

  CLI::App* parse = app.add_subcommand(
      "parse-route", "Validate and expand a filed route string (reverse of route)");
  parse
      ->add_option("route", a->route_str,
                   "Filed route string, e.g. \"KJFK SID CANDR J60 PSB ... STAR KLAX\"")
      ->required();
  parse->add_option("--data", a->data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  parse->add_option("--db", a->db_path, "Prebuilt .bfdb cache to load (skips parsing)");
  parse->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  parse->callback([a, &exit_code]() {
    Result<NavDatabase> db = OpenForRead(a->db_path, a->data_dir);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    Result<Route> result = db.value().ParseRoute(a->route_str);
    const auto elapsed_ms =
        static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now() - start)
                                  .count());
    if (!result) {
      std::cerr << "error: " << result.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    if (a->format == "json") {
      PrintRoutesJson({result.value()}, elapsed_ms);
    } else {
      PrintText(result.value());
      std::cout << "Query elapsed: " << elapsed_ms << " ms\n";
    }
  });
}

}  // namespace bf::cli
