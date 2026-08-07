// SPDX-License-Identifier: MIT
#include <CLI/CLI.hpp>
#include <cstdlib>
#include <string>

#include "commands.h"
#include "version_banner.h"

int main(int argc, char** argv) {
  CLI::App app{"BravoFinder CLI tools"};
  app.require_subcommand(1);
  // Shared LGPL §4c combined-work notice (see version_banner.h).
  app.set_version_flag("--version", bf::service::VersionBanner());

  int exit_code = EXIT_SUCCESS;
  bf::cli::RegisterBuild(app, exit_code);
  bf::cli::RegisterRoute(app, exit_code);
  bf::cli::RegisterQuery(app, exit_code);
  bf::cli::RegisterParseRoute(app, exit_code);

  CLI11_PARSE(app, argc, argv);
  return exit_code;
}
