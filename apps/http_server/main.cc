// bf-http: a hand-rolled HTTP query server exposing BravoFinder's route-finding
// and navigation-data lookups over HTTP+JSON, for an internal (Go) gateway.
//
// This file is the entry point only: it parses the CLI, scans a directory of
// `.bfdb` caches into a NavDatabaseRegistry (fail-fast if it holds none), sets
// the libuv threadpool size, then binds a TCP listener and runs the loop. Every
// connection's protocol handling lives in conn.cc; routing in router.cc; the
// query offload in work.cc. One loop thread does all I/O; the 10-30 ms route
// computation is offloaded to the threadpool, so a single loop scales.

#include <uv.h>

#include <CLI/CLI.hpp>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "conn.h"
#include "core/env.h"
#include "io/cache/bfdb_inventory.h"
#include "registry.h"
#include "router.h"
#include "server.h"

int main(int argc, char** argv) {
  // libuv on Linux neither sets SO_NOSIGPIPE nor passes MSG_NOSIGNAL to its
  // write syscalls, so a uv_write to a socket the peer has RST'd would deliver
  // SIGPIPE and kill the process. Ignore it process-wide (standard for a
  // hand-rolled network server); write errors still surface via the write
  // callback. No-op on Windows, which has no SIGPIPE.
#ifdef SIGPIPE
  std::signal(SIGPIPE, SIG_IGN);
#endif

  CLI::App app{"BravoFinder HTTP query server"};

  const char* env_dir = bf::GetEnv("BRAVOFINDER_NAVDATA");
  std::string db_dir = env_dir != nullptr ? env_dir : "navdata";
  std::string host = "0.0.0.0";
  int port = 8080;
  int worker_threads = static_cast<int>(std::thread::hardware_concurrency());
  if (worker_threads <= 0) {
    worker_threads = 4;
  }
  uint64_t max_body = 1u << 20;  // 1 MiB
  int io_timeout_sec = 30;

  app.add_option("--db-dir", db_dir, "Directory of nav_<cycle>.bfdb caches")->capture_default_str();
  app.add_option("--host", host, "Bind address")->capture_default_str();
  app.add_option("--port", port, "Bind port")->capture_default_str();
  app.add_option("--worker-threads", worker_threads,
                 "libuv threadpool size (UV_THREADPOOL_SIZE); route computation runs here")
      ->capture_default_str();
  app.add_option("--max-body", max_body, "Maximum request body size, in bytes")
      ->capture_default_str();
  app.add_option("--io-timeout", io_timeout_sec,
                 "Header/body read and idle keep-alive timeout, in seconds")
      ->capture_default_str();
  CLI11_PARSE(app, argc, argv);

  // libuv reads UV_THREADPOOL_SIZE once, the first time the pool is used, so set
  // it before the loop runs. Bounded to libuv's maximum of 1024.
  if (worker_threads > 1024) {
    worker_threads = 1024;
  }
  bf::SetEnv("UV_THREADPOOL_SIZE", std::to_string(worker_threads));

  // Build the registry (fail-fast, matching bf-mcp-stdio).
  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(db_dir);
  if (!inventory) {
    std::cerr << "error: " << inventory.error().message << "\n";
    return EXIT_FAILURE;
  }
  if (inventory.value().empty()) {
    std::cerr << "error: no nav_<cycle>.bfdb caches found in '" << db_dir
              << "' (build one with `bf build`, or set --db-dir / BRAVOFINDER_NAVDATA)\n";
    return EXIT_FAILURE;
  }
  for (const std::string& skipped : inventory.value().skipped()) {
    std::cerr << "warning: ignoring unreadable cache '" << skipped << "'\n";
  }
  const size_t cycle_count = inventory.value().entries().size();
  bf::service::NavDatabaseRegistry registry(std::move(inventory.value()));

  // Set up the loop, the router, and the TCP listener.
  uv_loop_t loop;
  uv_loop_init(&loop);
  bf::http::Router router(registry, &loop);

  bf::http::Limits limits;
  limits.max_body_bytes = static_cast<size_t>(max_body);
  limits.io_timeout_ms = static_cast<uint64_t>(io_timeout_sec) * 1000;

  bf::http::Server server(&loop, router, limits);
  const int rc = server.Listen(host, port);
  if (rc != 0) {
    std::cerr << "error: cannot listen on " << host << ":" << port << ": " << uv_strerror(rc)
              << "\n";
    return EXIT_FAILURE;
  }

  std::cerr << "bf-http listening on " << host << ":" << port << " (" << cycle_count
            << " cycle(s), " << worker_threads << " worker threads)\n";

  // Graceful shutdown: SIGINT / SIGTERM stop the loop so uv_run returns and main
  // exits cleanly (lets in-flight I/O drain a final iteration) instead of being
  // killed mid-write. SIGINT works on Windows; SIGTERM is POSIX-only.
  uv_signal_t sigint;
  uv_signal_init(&loop, &sigint);
  uv_signal_start(&sigint, [](uv_signal_t* h, int) { uv_stop(h->loop); }, SIGINT);
#ifndef _WIN32
  uv_signal_t sigterm;
  uv_signal_init(&loop, &sigterm);
  uv_signal_start(&sigterm, [](uv_signal_t* h, int) { uv_stop(h->loop); }, SIGTERM);
#endif

  uv_run(&loop, UV_RUN_DEFAULT);
  return 0;
}
