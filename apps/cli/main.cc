#include <CLI/CLI.hpp>
#include <cstdlib>

#include "commands.h"
#include "core/version.h"

int main(int argc, char** argv) {
  CLI::App app{"BravoFinder - a flight route finder"};
  app.require_subcommand(1);
  app.set_version_flag("--version", bf::kBravoFinderVersion);

  // Each subcommand registers its options and a callback; CLI11 invokes the
  // callback of the selected subcommand during parse. The callbacks write the
  // process exit code here.
  int exit_code = EXIT_SUCCESS;
  bf::cli::RegisterBuild(app, exit_code);
  bf::cli::RegisterRoute(app, exit_code);
  bf::cli::RegisterQuery(app, exit_code);
  bf::cli::RegisterParseRoute(app, exit_code);

  CLI11_PARSE(app, argc, argv);
  return exit_code;
}
