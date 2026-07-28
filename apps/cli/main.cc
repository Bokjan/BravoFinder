// SPDX-License-Identifier: MIT
#include <CLI/CLI.hpp>
#include <cstdlib>
#include <string>

#include "commands.h"
#include "core/version.h"

int main(int argc, char** argv) {
  CLI::App app{"BravoFinder - a flight route finder"};
  app.require_subcommand(1);
  // The route engine (libs/engine) is statically linked and covered by the
  // LGPL, so --version carries the Library copyright notice and points to the
  // GPL/LGPL text, satisfying LGPLv3 section 4c for this combined work.
  app.set_version_flag(
      "--version",
      std::string("BravoFinder ") + bf::kBravoFinderVersion + "\n" + "Copyright (c) Boyin Chen\n" +
          "MIT-licensed, except the route engine (libs/engine/) which is under the GNU LGPL "
          "v3.0-or-later.\n" +
          "See LICENSE.md, LICENSE.MIT, libs/engine/LICENSE and libs/engine/LICENSE.GPLv3.");

  int exit_code = EXIT_SUCCESS;
  bf::cli::RegisterBuild(app, exit_code);
  bf::cli::RegisterRoute(app, exit_code);
  bf::cli::RegisterQuery(app, exit_code);
  bf::cli::RegisterParseRoute(app, exit_code);

  CLI11_PARSE(app, argc, argv);
  return exit_code;
}
