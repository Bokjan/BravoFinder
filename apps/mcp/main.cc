// bf-mcp: a local stdio MCP server exposing BravoFinder's route-finding
// and navigation-data lookup as MCP tools.
//
// This file is only the entry point: it scans a directory of `.bfdb` caches
// into a NavDatabaseRegistry (fail-fast if the directory holds none), then runs
// an McpServer over stdio. Databases are opened lazily per AIRAC cycle on first
// use. The protocol lives in mcp_server.cc; the capabilities live in tools.cc;
// the registry in registry.cc.

#include <CLI/CLI.hpp>
#include <iostream>
#include <string>

#include "core/env.h"
#include "core/version.h"
#include "io/cache/bfdb_inventory.h"
#include "mcp_server.h"
#include "registry.h"

int main(int argc, char** argv) {
  const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA");
  std::string db_dir = env ? env : "navdata";

  // Parse CLI options. --version (and parse errors) are handled by CLI11 and
  // exit before any cache scan, so `bf-mcp --version` works with no data.
  CLI::App app{"BravoFinder MCP server"};
  app.add_option("--db-dir", db_dir, "Directory of nav_<cycle>.bfdb caches")->capture_default_str();
  app.set_version_flag("--version", bf::kBravoFinderVersion);
  CLI11_PARSE(app, argc, argv);
  const std::string dir = db_dir;

  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(dir);
  if (!inventory) {
    std::cerr << "error: " << inventory.error().message << "\n";
    return EXIT_FAILURE;
  }
  if (inventory.value().empty()) {
    std::cerr << "error: no nav_<cycle>.bfdb caches found in '" << dir
              << "' (build one with `bf build`, or set --db-dir / BRAVOFINDER_NAVDATA)\n";
    return EXIT_FAILURE;
  }
  // Surface files that were present but unusable, so reduced coverage is not
  // silent.
  for (const std::string& skipped : inventory.value().skipped()) {
    std::cerr << "warning: ignoring unreadable cache '" << skipped << "'\n";
  }

  bf::service::NavDatabaseRegistry registry(std::move(inventory.value()));
  bf::mcp::McpServer server(registry);
  return server.Run();
}
