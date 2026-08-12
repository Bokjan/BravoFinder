// SPDX-License-Identifier: MIT
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
#include <string>
#include <thread>

#include "core/base/env.h"
#include "io/cache/bfdb_inventory.h"
#include "io_print.h"
#include "registry.h"
#include "router.h"
#include "server.h"
#include "version_banner.h"

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
  std::string host = "127.0.0.1";
  int port = bf::http_server::kDefaultPort;
  int worker_threads = static_cast<int>(std::thread::hardware_concurrency());
  if (worker_threads <= 0) {
    worker_threads = 4;
  }
  uint64_t max_body = bf::http_server::kDefaultMaxBodyBytes;
  int io_timeout_sec = bf::http_server::kDefaultIoTimeoutSec;
  std::string cifp_load = "on-demand";

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
  app.add_option("--cifp-load", cifp_load,
                 "CIFP procedure loading: on-demand (low memory) or eager "
                 "(lock-free reads, ~100 MB per cycle)")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
  // Shared LGPL §4c combined-work notice (see version_banner.h).
  app.set_version_flag("--version", bf::service::VersionBanner());
  CLI11_PARSE(app, argc, argv);

  // Reject out-of-range bind / pool / timeout values with a clear stderr message
  // rather than clamping (silent clamp hid misconfiguration).
  if (port < 1 || port > 65535) {
    bf::service::PrintError("--port must be in 1..65535 (got {})", port);
    return EXIT_FAILURE;
  }
  if (worker_threads < 1 || worker_threads > 1024) {
    bf::service::PrintError("--worker-threads must be in 1..1024 (got {})", worker_threads);
    return EXIT_FAILURE;
  }
  if (io_timeout_sec < 1) {
    bf::service::PrintError("--io-timeout must be >= 1 second (got {})", io_timeout_sec);
    return EXIT_FAILURE;
  }

  // libuv reads UV_THREADPOOL_SIZE once, the first time the pool is used, so set
  // it before the loop runs.
  bf::SetEnv("UV_THREADPOOL_SIZE", std::to_string(worker_threads));

  // Build the registry (fail-fast, matching bf-mcp).
  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(db_dir);
  if (!inventory) {
    bf::service::PrintError(inventory.error().message);
    return EXIT_FAILURE;
  }
  if (inventory.value().empty()) {
    bf::service::PrintError(
        "no nav_<cycle>.bfdb caches found in '{}' (build one with `bf build`, or set "
        "--db-dir / BRAVOFINDER_NAVDATA)",
        db_dir);
    return EXIT_FAILURE;
  }
  for (const std::string& skipped : inventory.value().skipped()) {
    bf::service::PrintWarning("ignoring unreadable cache '{}'", skipped);
  }
  const size_t cycle_count = inventory.value().entries().size();
  const auto cifp_mode = cifp_load == "eager" ? bf::CifpLoad::kEager : bf::CifpLoad::kOnDemand;
  bf::service::NavDatabaseRegistry registry(std::move(inventory.value()), cifp_mode);

  // Set up the loop, the router, and the TCP listener.
  uv_loop_t loop;
  uv_loop_init(&loop);
  bf::http::Router router(registry, &loop);

  bf::http_server::Limits limits;
  limits.max_body_bytes = static_cast<size_t>(max_body);
  limits.io_timeout_ms = static_cast<uint64_t>(io_timeout_sec) * bf::http_server::kMsPerSec;

  bf::http_server::Server server(&loop, router, limits);
  const int rc = server.Listen(host, port);
  if (rc != 0) {
    bf::service::PrintError("cannot listen on {}:{}: {}", host, port, uv_strerror(rc));
    return EXIT_FAILURE;
  }

  bf::service::PrintStatus("bf-http listening on {}:{} ({} cycle(s), {} worker threads)", host,
                           port, cycle_count, worker_threads);

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

  // Graceful drain after uv_stop. Route-compute work may still be in flight on
  // the threadpool; its completion callbacks (which read the registry) must run
  // before `registry` — a stack local below — is destroyed. Shutdown() stops
  // accepting and closes every live Connection through Connection::Close
  // (StartClose), so self_ resets and IsAlive() goes false. A bare uv_walk that
  // uv_close()s Connection handles with a null callback would bypass StartClose
  // and leave connections looking alive. The walk below only closes leftover
  // non-connection handles (signal watchers, etc.); already-closing handles are
  // skipped by uv_is_closing. The pending uv_work_t requests keep the loop alive
  // until they finish, so this final run delivers their completions and every
  // handle's close callback before we exit. Mirrors http_test.cc teardown.
  server.Shutdown();
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
    // Nonzero (UV_EBUSY) means a handle outlived the drain above -- harmless at
    // process exit, but a signal that the teardown missed something, so surface
    // it rather than swallow a future drain regression.
    bf::service::PrintWarning("uv_loop_close: {} (a handle outlived the drain)",
                              uv_strerror(close_rc));
  }
  return 0;
}
