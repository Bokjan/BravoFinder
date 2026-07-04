#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "commands.h"
#include "io/cache/bfdb_naming.h"
#include "io/nav_database.h"

namespace bf::cli {

void RegisterBuild(CLI::App& app, int& exit_code) {
  struct Args {
    std::string data_dir;
    std::string output;
    std::string loader = "xplane12";
    bool without_cifp = false;
  };
  auto args = std::make_shared<Args>();

  CLI::App* build = app.add_subcommand("build", "Build a .bfdb cache from X-Plane data");
  build->add_option("data_dir", args->data_dir, "Directory of X-Plane navigation data")->required();
  build->add_option("-o,--output", args->output,
                    "Output .bfdb path (default: <data_dir>/nav_<cycle>.bfdb)");
  build->add_option("--loader", args->loader, "Data source loader")
      ->capture_default_str()
      ->check(CLI::IsMember({"xplane12"}));
  build->add_flag("--without-cifp", args->without_cifp,
                  "Omit the CIFP procedure section from the .bfdb");

  build->callback([args, &exit_code]() {
    Result<NavDatabase> db = NavDatabase::Open(args->data_dir, args->loader);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    // Default output name encodes the AIRAC cycle parsed from the data, so a
    // directory of caches can be told apart and served by the MCP registry. An
    // explicit -o is honored verbatim.
    const std::string out = args->output.empty()
                                ? args->data_dir + "/" + FormatBfdbName(db.value().cycle())
                                : args->output;
    // One unified file holds the graph, the CIFP procedures, and the navaid
    // detail. --without-cifp omits the CIFP section (smaller file, no procedures).
    Result<uint32_t> written = db.value().WriteUnified(out, !args->without_cifp);
    if (!written) {
      std::cerr << "error: " << written.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    std::cout << "wrote " << out << " (" << written.value() << " airports with procedures)\n";
  });
}

}  // namespace bf::cli
