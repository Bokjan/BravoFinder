// SPDX-License-Identifier: MIT
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "cli_common.h"
#include "commands.h"
#include "io_print.h"
#include "queries.h"
#include "render.h"

namespace bf::cli {

void RegisterQuery(CLI::App& app, int& exit_code) {
  struct Args {
    std::string kind;
    std::vector<std::string> ids;
    std::string data_dir = "navdata";
    std::string db_path;
    std::string cifp_load = "on-demand";
    std::string format = "text";
  };
  auto a = std::make_shared<Args>();

  CLI::App* query = app.add_subcommand(
      "query", "Look up navigation data (waypoints, airports, procedures, airways)");
  query
      ->add_option("kind", a->kind,
                   "What to look up: waypoint, airport, procedure, airway, navaid_detail, or hold")
      ->required()
      ->check(
          CLI::IsMember({"waypoint", "airport", "procedure", "airway", "navaid_detail", "hold"}));
  query
      ->add_option("id", a->ids,
                   "One or more idents / ICAO codes / airway names. For 'procedure', an "
                   "ICAO/NAME selector (e.g. KJFK/DEEZZ5) prints that procedure's per-leg detail")
      ->required();
  query->add_option("--data", a->data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  query->add_option("--db", a->db_path, "Prebuilt .bfdb cache to load (skips parsing)");
  query
      ->add_option("--cifp-load", a->cifp_load,
                   "CIFP cache load mode: on-demand (default) or eager")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
  query->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  query->callback([a, &exit_code]() {
    Result<NavDatabase> db = OpenForRead(a->db_path, a->data_dir, a->cifp_load);
    if (!db) {
      bf::service::PrintError(db.error().message);
      exit_code = EXIT_FAILURE;
      return;
    }

    const bf::service::OutputFormat fmt =
        a->format == "json" ? bf::service::OutputFormat::kJson : bf::service::OutputFormat::kText;

    // Delegate to the shared query layer. Each batch lookup returns a body
    // parallel to `ids` and a status: 200 on a partial or full hit, 404 when
    // every id missed. The `procedure` kind takes a mix of selectors (a bare
    // airport lists its procedure summaries; an "airport/procedure" pair prints
    // that named procedure's per-leg detail).
    bf::service::HandlerResult result;
    if (a->kind == "waypoint") {
      result = bf::service::LookupWaypoints(db.value(), a->ids, fmt);
    } else if (a->kind == "airport") {
      result = bf::service::LookupAirports(db.value(), a->ids, fmt);
    } else if (a->kind == "airway") {
      result = bf::service::LookupAirways(db.value(), a->ids, fmt);
    } else if (a->kind == "navaid_detail") {
      result = bf::service::LookupNavaidDetails(db.value(), a->ids, fmt);
    } else if (a->kind == "hold") {
      result = bf::service::LookupHolds(db.value(), a->ids, fmt);
    } else {  // procedure
      std::vector<bf::service::ProcedureSelector> selectors;
      selectors.reserve(a->ids.size());
      for (const std::string& id : a->ids) {
        const size_t slash = id.find('/');
        if (slash == std::string::npos) {
          selectors.push_back({id, ""});
        } else {
          selectors.push_back({id.substr(0, slash), id.substr(slash + 1)});
        }
      }
      result = bf::service::LookupProceduresMixed(db.value(), selectors, fmt);
    }

    // Match cmd_route / cmd_parse_route: failures (status >= 400) go to stderr
    // with a non-zero exit; success stays on stdout. A partial hit is still
    // success (status 200) and prints the parallel body to stdout.
    if (result.status >= bf::service::kErrorStatusThreshold) {
      bf::service::PrintStderr(result.body,
                               /*ensure_newline=*/fmt == bf::service::OutputFormat::kJson);
      exit_code = EXIT_FAILURE;
      return;
    }
    bf::service::PrintStdout(result.body,
                             /*ensure_newline=*/fmt == bf::service::OutputFormat::kJson);
  });
}

}  // namespace bf::cli
