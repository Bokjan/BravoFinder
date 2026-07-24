// bf-mcp: an MCP server exposing BravoFinder's route-finding and navigation-data
// lookup as MCP tools, over one of two transports:
//
//   * stdio (default): JSON-RPC over stdin/stdout, for a local MCP client that
//     spawns the process. This is the backward-compatible default.
//   * http: MCP-over-HTTP (Streamable HTTP, 2025-03-26) on a TCP port, for
//     remote / multi-client MCP clients. Built on the same shared HTTP core
//     (http_server/) as the REST server, so the 10-30 ms route compute is
//     offloaded to the libuv threadpool and never blocks the loop.
//
// Both transports share the same Dispatcher (protocol + capabilities); this file
// only scans a directory of `.bfdb` caches into a NavDatabaseRegistry (fail-fast
// if it holds none), then wires up the chosen transport. Databases open lazily
// per AIRAC cycle on first use. The protocol lives in dispatcher.cc; the stdio
// transport in stdio_runner.cc; the HTTP transport in mcp_http.cc; the
// capabilities in tools.cc; the registry in registry.cc.

#include <uv.h>

#include <CLI/CLI.hpp>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "core/env.h"
#include "core/version.h"
#include "io/cache/bfdb_inventory.h"
#include "mcp_http.h"
#include "registry.h"
#include "server.h"
#include "stdio_runner.h"

namespace {

// Run the MCP-over-HTTP transport: bind a TCP listener and run the libuv loop
// until SIGINT/SIGTERM, then drain in-flight work so no worker touches the
// registry after it is destroyed. Mirrors apps/http/main.cc's loop/signal/drain
// idiom. Returns the process exit status.
int RunHttp(bf::service::NavDatabaseRegistry& registry, const std::string& host, int port,
            int worker_threads, uint64_t max_body, int io_timeout_sec, size_t cycle_count) {
  // libuv on Linux neither sets SO_NOSIGPIPE nor passes MSG_NOSIGNAL, so a
  // uv_write to a peer-RST'd socket would deliver SIGPIPE and kill the process.
  // Ignore it process-wide; write errors still surface via the write callback.
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif
  // libuv reads UV_THREADPOOL_SIZE once, the first time the pool is used, so set
  // it before the loop runs. Bounded to libuv's maximum of 1024.
  if (worker_threads > 1024) {
    worker_threads = 1024;
  }
  bf::SetEnv("UV_THREADPOOL_SIZE", std::to_string(worker_threads));

  uv_loop_t loop;
  uv_loop_init(&loop);
  bf::mcp::McpHttpHandler handler(registry, &loop);

  bf::http_server::Limits limits;
  limits.max_body_bytes = static_cast<size_t>(max_body);
  limits.io_timeout_ms = static_cast<uint64_t>(io_timeout_sec) * 1000;

  bf::http_server::Server server(&loop, handler, limits);
  const int rc = server.Listen(host, port);
  if (rc != 0) {
    std::cerr << "error: cannot listen on " << host << ":" << port << ": " << uv_strerror(rc)
              << "\n";
    if (const int close_rc = uv_loop_close(&loop); close_rc != 0) {
      std::cerr << "warning: uv_loop_close: " << uv_strerror(close_rc) << "\n";
    }
    return EXIT_FAILURE;
  }
  std::cerr << "bf-mcp (http) listening on " << host << ":" << port << " (" << cycle_count
            << " cycle(s), " << worker_threads << " worker threads)\n";

  uv_signal_t sigint;
  uv_signal_init(&loop, &sigint);
  uv_signal_start(&sigint, [](uv_signal_t* h, int) { uv_stop(h->loop); }, SIGINT);
#ifndef _WIN32
  uv_signal_t sigterm;
  uv_signal_init(&loop, &sigterm);
  uv_signal_start(&sigterm, [](uv_signal_t* h, int) { uv_stop(h->loop); }, SIGTERM);
#endif

  uv_run(&loop, UV_RUN_DEFAULT);

  // Graceful drain: close every remaining handle and run the loop again so any
  // in-flight route-compute completions (which read the registry) run before the
  // registry -- a caller's stack local -- is destroyed.
  uv_walk(
      &loop,
      [](uv_handle_t* h, void*) {
        if (uv_is_closing(h) == 0) {
          uv_close(h, nullptr);
        }
      },
      nullptr);
  uv_run(&loop, UV_RUN_DEFAULT);
  if (const int close_rc = uv_loop_close(&loop); close_rc != 0) {
    // Nonzero (UV_EBUSY) means a handle outlived the drain -- harmless at exit,
    // but surfaced rather than swallowed so a future drain regression is visible.
    std::cerr << "warning: uv_loop_close: " << uv_strerror(close_rc)
              << " (a handle outlived the drain)\n";
  }
  return EXIT_SUCCESS;
}

}  // namespace

int main(int argc, char** argv) {
  const char* env = bf::GetEnv("BRAVOFINDER_NAVDATA");
  std::string db_dir = env ? env : "navdata";

  std::string transport = "stdio";
  std::string host = "0.0.0.0";
  int port = 8080;
  int worker_threads = static_cast<int>(std::thread::hardware_concurrency());
  if (worker_threads <= 0) {
    worker_threads = 4;
  }
  uint64_t max_body = 1u << 20;  // 1 MiB
  int io_timeout_sec = bf::http_server::kDefaultIoTimeoutSec;
  std::string cifp_load = "on-demand";

  // Parse CLI options. --version (and parse errors) are handled by CLI11 and
  // exit before any cache scan, so `bf-mcp --version` works with no data.
  CLI::App app{"BravoFinder MCP server"};
  app.add_option("--db-dir", db_dir, "Directory of nav_<cycle>.bfdb caches")->capture_default_str();
  app.add_option("--transport", transport, "MCP transport: stdio (default) or http")
      ->capture_default_str()
      ->check(CLI::IsMember({"stdio", "http"}));
  // HTTP-only options; ignored in stdio mode.
  app.add_option("--host", host, "Bind address (http transport)")->capture_default_str();
  app.add_option("--port", port, "Bind port (http transport)")->capture_default_str();
  app.add_option("--worker-threads", worker_threads,
                 "libuv threadpool size (http transport); route computation runs here")
      ->capture_default_str();
  app.add_option("--max-body", max_body, "Maximum request body size in bytes (http transport)")
      ->capture_default_str();
  app.add_option("--io-timeout", io_timeout_sec,
                 "Header/body read and idle keep-alive timeout in seconds (http transport)")
      ->capture_default_str();
  app.add_option("--cifp-load", cifp_load,
                 "CIFP procedure loading: on-demand (low memory) or eager "
                 "(lock-free reads, ~100 MB per cycle)")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
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
  const size_t cycle_count = inventory.value().entries().size();
  const auto cifp_mode =
      cifp_load == "eager" ? bf::CifpLoad::kEager : bf::CifpLoad::kOnDemand;
  bf::service::NavDatabaseRegistry registry(std::move(inventory.value()), cifp_mode);

  if (transport == "http") {
    return RunHttp(registry, host, port, worker_threads, max_body, io_timeout_sec, cycle_count);
  }
  bf::mcp::StdioRunner runner(registry);
  return runner.Run();
}
