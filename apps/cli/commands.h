// SPDX-License-Identifier: MIT
#pragma once

#include <CLI/CLI.hpp>

namespace bf::cli {

// Each RegisterX adds its subcommand to `app` and wires a callback that runs
// when that subcommand is selected, writing the process exit code to
// `exit_code`. Args are owned by the callback (heap-allocated, shared into the
// lambda), so they outlive parsing. main() only calls these, parses, and
// returns exit_code.
void RegisterBuild(CLI::App& app, int& exit_code);
void RegisterRoute(CLI::App& app, int& exit_code);
void RegisterQuery(CLI::App& app, int& exit_code);
void RegisterParseRoute(CLI::App& app, int& exit_code);

}  // namespace bf::cli
