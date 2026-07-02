#include "commands.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "io/nav_database.h"

namespace bf::cli {

void RegisterBuild(CLI::App& app, int& exit_code) {
  struct Args {
    std::string data_dir;
    std::string output;
    std::string loader = "xplane";
    bool without_cifp = false;
  };
  auto args = std::make_shared<Args>();

  CLI::App* build = app.add_subcommand("build", "Build a .bfdb cache from X-Plane data");
  build->add_option("data_dir", args->data_dir, "Directory of X-Plane navigation data")->required();
  build->add_option("-o,--output", args->output,
                    "Output .bfdb path (default: <data_dir>/nav.bfdb)");
  build->add_option("--loader", args->loader, "Data source loader")
      ->capture_default_str()
      ->check(CLI::IsMember({"xplane"}));
  build->add_flag("--without-cifp", args->without_cifp,
                  "Skip building the CIFP procedure cache (<stem>_cifp.bfdb)");

  build->callback([args, &exit_code]() {
    const std::string out = args->output.empty() ? args->data_dir + "/nav.bfdb" : args->output;
    Result<NavDatabase> db = NavDatabase::Open(args->data_dir);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
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
      Result<uint32_t> n = db.value().WriteCifpCache(cifp_out, args->loader);
      if (!n) {
        std::cerr << "error: " << n.error().message << "\n";
        exit_code = EXIT_FAILURE;
        return;
      }
      std::cout << "wrote " << cifp_out << " (" << n.value() << " airports)\n";
    }
  });
}

}  // namespace bf::cli
