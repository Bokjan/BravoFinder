// SPDX-License-Identifier: MIT
#include <CLI/CLI.hpp>
#include <cstdlib>
#include <memory>
#include <string>

#include "commands.h"
#include "core/base/log.h"
#include "version_banner.h"

int main(int argc, char** argv) {
  // Engine BF_LOG_* is silent until a sink is installed. CLI installs stderr at
  // WARN so loader/diagnostics warnings surface; user-facing failures stay on
  // PrintCliError ("error: ...") and do not go through BF_LOG_*.
  auto logger = std::make_shared<bf::StderrLogger>();
  logger->set_min_level(bf::LogLevel::kWarn);
  bf::SetDefaultLogger(logger);

  CLI::App app{"BravoFinder CLI tools"};
  app.require_subcommand(1);
  // Shared LGPL §4c combined-work notice (see version_banner.h).
  app.set_version_flag("--version", bf::service::VersionBanner());
  // each() runs when the flag is parsed -- before the subcommand callback -- so
  // Open/build logs already see DEBUG if -v was on the command line.
  app.add_flag("-v,--verbose", "Enable engine debug logging on stderr (default: warn)")
      ->each([&logger](const std::string&) { logger->set_min_level(bf::LogLevel::kDebug); });

  int exit_code = EXIT_SUCCESS;
  bf::cli::RegisterBuild(app, exit_code);
  bf::cli::RegisterRoute(app, exit_code);
  bf::cli::RegisterQuery(app, exit_code);
  bf::cli::RegisterParseRoute(app, exit_code);

  CLI11_PARSE(app, argc, argv);
  return exit_code;
}
