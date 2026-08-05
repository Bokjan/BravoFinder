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
#include <format>
#include <iostream>
#include <string>
#include <thread>

#include "core/base/env.h"
#include "core/version.h"
#include "io/cache/bfdb_inventory.h"
#include "registry.h"
#include "render.h"
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
  // The route engine (libs/engine) is statically linked and LGPL-covered, so
  // --version carries the Library copyright notice and points to the GPL/LGPL
  // text, satisfying LGPLv3 section 4c for this combined work.
  app.set_version_flag(
      "--version",
      std::format(
          "BravoFinder {}\n"
          "Copyright (c) Boyin Chen, and all contributors\n"
          "MIT-licensed, except the route engine (libs/engine/) which is under the GNU LGPL "
          "v3.0-or-later.\n"
          "See LICENSE.md, LICENSE.MIT, libs/engine/LICENSE and libs/engine/LICENSE.GPLv3.",
          bf::kBravoFinderVersion));
  CLI11_PARSE(app, argc, argv);

  // libuv reads UV_THREADPOOL_SIZE once, the first time the pool is used, so set
  // it before the loop runs. Bounded to libuv's maximum of 1024.
  if (worker_threads > 1024) {
    worker_threads = 1024;
  }
  bf::SetEnv("UV_THREADPOOL_SIZE", std::to_string(worker_threads));

  // Build the registry (fail-fast, matching bf-mcp).
  bf::Result<bf::BfdbInventory> inventory = bf::BfdbInventory::Scan(db_dir);
  if (!inventory) {
    std::cerr << bf::service::kTextErrorPrefix << inventory.error().message << "\n";
    return EXIT_FAILURE;
  }
  if (inventory.value().empty()) {
    std::cerr << bf::service::kTextErrorPrefix << "no nav_<cycle>.bfdb caches found in '" << db_dir
              << "' (build one with `bf build`, or set --db-dir / BRAVOFINDER_NAVDATA)\n";
    return EXIT_FAILURE;
  }
  for (const std::string& skipped : inventory.value().skipped()) {
    std::cerr << "warning: ignoring unreadable cache '" << skipped << "'\n";
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
    std::cerr << bf::service::kTextErrorPrefix << "cannot listen on " << host << ":" << port << ": "
              << uv_strerror(rc) << "\n";
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

  // Graceful drain. uv_stop returned the loop while route-compute work may still
  // be in flight on the threadpool; its completion callbacks (which read the
  // registry) must run before `registry` -- a stack local below -- is destroyed,
  // or a worker could touch it in the exit window. Close every remaining handle
  // (listener, live connections, signal watchers) and run the loop again: the
  // pending uv_work_t requests keep it alive until they finish, so this final run
  // delivers their completions and every handle's close callback before we exit.
  // Mirrors the teardown idiom in http_test.cc.
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
    std::cerr << "warning: uv_loop_close: " << uv_strerror(close_rc)
              << " (a handle outlived the drain)\n";
  }
  return 0;
}
