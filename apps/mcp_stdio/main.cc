// bf-mcp-stdio: a local stdio MCP server exposing BravoFinder's route-finding
// and navigation-data lookup as MCP tools.
//
// This file is only the entry point: it resolves and loads the .bfdb cache
// once at startup (fail-fast if missing/corrupt), then hands the read-only
// database to an McpServer, which owns the stdio JSON-RPC loop. The protocol
// lives in mcp_server.cc; the capabilities live in tools.cc.

#include <cstdlib>
#include <memory>
#include <string>

#include "io/nav_database.h"
#include "mcp_server.h"

namespace {

struct StartupOpts {
  std::string data_dir;
  std::string db_path;
  std::string cifp_db_path;
};

// Resolve startup options: BRAVOFINDER_NAVDATA sets the default data directory;
// --db / --cifp-db / --data override it. Accepts both "--flag value" and
// "--flag=value" forms.
StartupOpts ParseStartupArgs(int argc, char** argv) {
  StartupOpts opts;
  const char* env = std::getenv("BRAVOFINDER_NAVDATA");
  opts.data_dir = env ? env : "navdata";
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto take_value = [&](const std::string& flag, std::string& out) {
      if (a == flag && i + 1 < argc) {
        out = argv[++i];
        return true;
      }
      if (a.rfind(flag + "=", 0) == 0) {
        out = a.substr(flag.size() + 1);
        return true;
      }
      return false;
    };
    if (take_value("--db", opts.db_path)) continue;
    if (take_value("--cifp-db", opts.cifp_db_path)) continue;
    if (take_value("--data", opts.data_dir)) continue;
  }
  return opts;
}

// Load the cache once. Returns nullptr (and prints to stderr) on failure; the
// caller then exits non-zero so the MCP client sees a clean startup failure.
std::unique_ptr<bf::NavDatabase> LoadDatabase(const StartupOpts& opts) {
  std::string db_path = opts.db_path;
  if (db_path.empty()) {
    db_path = opts.data_dir + "/nav.bfdb";
  }
  bf::Result<bf::NavDatabase> db =
      bf::NavDatabase::OpenCached(db_path, opts.data_dir, opts.cifp_db_path);
  if (!db) {
    std::cerr << "error: " << db.error().message << "\n";
    return nullptr;
  }
  return std::make_unique<bf::NavDatabase>(std::move(db.value()));
}

}  // namespace

int main(int argc, char** argv) {
  const StartupOpts opts = ParseStartupArgs(argc, argv);
  auto db = LoadDatabase(opts);
  if (!db) {
    return EXIT_FAILURE;
  }
  bf::mcp::McpServer server(*db);
  return server.Run();
}
