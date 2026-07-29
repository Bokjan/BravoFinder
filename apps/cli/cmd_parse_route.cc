// SPDX-License-Identifier: MIT
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "cli_common.h"
#include "commands.h"
#include "queries.h"
#include "render.h"

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
    const bf::service::OutputFormat fmt =
        a->format == "json" ? bf::service::OutputFormat::kJson : bf::service::OutputFormat::kText;
    const bf::service::HandlerResult result =
        bf::service::ParseRoute(db.value(), a->route_str, fmt);
    if (result.status >= bf::service::kErrorStatusThreshold) {
      std::cerr << result.body;
      if (fmt == bf::service::OutputFormat::kJson) {
        std::cerr << "\n";
      }
      exit_code = EXIT_FAILURE;
      return;
    }
    if (fmt == bf::service::OutputFormat::kJson) {
      // The query layer renders a single route object for parse-route (the
      // transport shape); the CLI wraps it with the elapsed_ms envelope it has
      // shipped since v3.13.0. parse_route yields exactly one route, so the
      // envelope uses the singular "route" key (not the "routes" array).
      std::cout << bf::cli::WrapRouteEnvelope(result.body, result.elapsed_ms) << "\n";
    } else {
      std::cout << result.body;
    }
  });
}

}  // namespace bf::cli
