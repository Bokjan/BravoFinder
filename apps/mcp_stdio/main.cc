// bf-mcp-stdio: a local stdio MCP server exposing BravoFinder's route-finding
// and navigation-data lookup as MCP tools.
//
// This file is only the entry point: it scans a directory of `.bfdb` caches
// into a NavDatabaseRegistry (fail-fast if the directory holds none), then runs
// an McpServer over stdio. Databases are opened lazily per AIRAC cycle on first
// use. The protocol lives in mcp_server.cc; the capabilities live in tools.cc;
// the registry in registry.cc.

#include <iostream>
#include <string>

#include "core/env.h"
#include "io/cache/bfdb_inventory.h"
#include "mcp_server.h"
#include "registry.h"

namespace {

// Resolve the directory of `.bfdb` caches: --db-dir wins, else the
// BRAVOFINDER_NAVDATA environment variable, else "navdata". A single-cycle
// deployment is just a directory holding one nav_<cycle>_<build>.bfdb.
std::string ResolveDir(int argc, char** argv) {
  const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA");
  std::string dir = env ? env : "navdata";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--db-dir" && i + 1 < argc) {
      dir = argv[++i];
    } else if (a.rfind("--db-dir=", 0) == 0) {
      dir = a.substr(std::string("--db-dir=").size());
    }
  }
  return dir;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string dir = ResolveDir(argc, argv);

  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(dir);
  if (!inventory) {
    std::cerr << "error: " << inventory.error().message << "\n";
    return EXIT_FAILURE;
  }
  if (inventory.value().empty()) {
    std::cerr << "error: no nav_<cycle>_<build>.bfdb caches found in '" << dir
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
