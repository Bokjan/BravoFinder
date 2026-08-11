// SPDX-License-Identifier: MIT
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#include "cli_common.h"
#include "commands.h"
#include "io_print.h"
#include "queries.h"
#include "render.h"

namespace bf::cli {

void RegisterParseRoute(CLI::App& app, int& exit_code) {
  struct Args {
    std::string route_str;
    std::string data_dir = "navdata";
    std::string db_path;
    std::string cifp_load = "on-demand";
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
  parse
      ->add_option("--cifp-load", a->cifp_load,
                   "CIFP cache load mode: on-demand (default) or eager")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
  parse->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  parse->callback([a, &exit_code]() {
    Result<NavDatabase> db = OpenForRead(a->db_path, a->data_dir, a->cifp_load);
    if (!db) {
      bf::service::PrintError(db.error().message);
      exit_code = EXIT_FAILURE;
      return;
    }
    const bf::service::OutputFormat fmt =
        a->format == "json" ? bf::service::OutputFormat::kJson : bf::service::OutputFormat::kText;
    const bf::service::HandlerResult result =
        bf::service::ParseRoute(db.value(), a->route_str, fmt);
    if (result.status >= bf::service::kErrorStatusThreshold) {
      bf::service::PrintStderr(result.body,
                               /*ensure_newline=*/fmt == bf::service::OutputFormat::kJson);
      exit_code = EXIT_FAILURE;
      return;
    }
    if (fmt == bf::service::OutputFormat::kJson) {
      // The query layer renders a single route object for parse-route (the
      // transport shape); the CLI wraps it with the elapsed_ms envelope it has
      // shipped since v3.13.0. parse_route yields exactly one route, so the
      // envelope uses the singular "route" key (not the "routes" array).
      bf::service::PrintStdout(bf::cli::WrapRouteEnvelope(result.body, result.elapsed_ms),
                               /*ensure_newline=*/true);
    } else {
      bf::service::PrintStdout(result.body);
    }
  });
}

}  // namespace bf::cli
