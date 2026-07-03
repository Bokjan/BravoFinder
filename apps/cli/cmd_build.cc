#include <cstdlib>
#include <filesystem>
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
                    "Output .bfdb path (default: <data_dir>/nav_<cycle>_<build>.bfdb)");
  build->add_option("--loader", args->loader, "Data source loader")
      ->capture_default_str()
      ->check(CLI::IsMember({"xplane12"}));
  build->add_flag("--without-cifp", args->without_cifp,
                  "Skip building the CIFP procedure cache (<stem>_cifp.bfdb)");

  build->callback([args, &exit_code]() {
    Result<NavDatabase> db = NavDatabase::Open(args->data_dir, args->loader);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    // Default output name encodes the AIRAC cycle/build parsed from the data, so
    // a directory of caches can be told apart and served by the MCP registry. An
    // explicit -o is honored verbatim.
    const std::string out =
        args->output.empty()
            ? args->data_dir + "/" + FormatBfdbName(db.value().cycle(), db.value().build())
            : args->output;
    Result<void> written = db.value().WriteCache(out);
    if (!written) {
      std::cerr << "error: " << written.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    std::cout << "wrote " << out << "\n";

    // Also build the CIFP procedure cache unless opted out, so deployment needs
    // only the cache files. Its path mirrors the graph cache: <stem>_cifp.bfdb.
    if (!args->without_cifp) {
      const std::filesystem::path p(out);
      const std::string cifp_out = (p.parent_path() / (p.stem().string() + "_cifp.bfdb")).string();
      Result<uint32_t> n = db.value().WriteCifpCache(cifp_out);
      if (!n) {
        std::cerr << "error: " << n.error().message << "\n";
        exit_code = EXIT_FAILURE;
        return;
      }
      std::cout << "wrote " << cifp_out << " (" << n.value() << " airports)\n";
    }

    // Build the navaid-detail + hold side cache: <stem>_detail.bfdb. Small
    // (~3 MB) and always written, so navaid_detail / hold queries work off a
    // cache. NavDatabase::OpenCached auto-discovers it by this name.
    {
      const std::filesystem::path p(out);
      const std::string detail_out =
          (p.parent_path() / (p.stem().string() + "_detail.bfdb")).string();
      Result<void> d = db.value().WriteDetailCache(detail_out);
      if (!d) {
        std::cerr << "error: " << d.error().message << "\n";
        exit_code = EXIT_FAILURE;
        return;
      }
      std::cout << "wrote " << detail_out << "\n";
    }
  });
}

}  // namespace bf::cli
