// SPDX-License-Identifier: MIT
// server.h — the TCP listener + accept loop for the shared HTTP core.
//
// Server owns the listening uv_tcp_t and turns each accepted socket into a
// Connection routed to a RequestHandler. It is extracted so tests (and each
// app's main) can start a real listener on a loopback port (Listen with port 0,
// then BoundPort) and drive it over an actual socket. All methods run on the
// listener's loop thread.

#pragma once

#include <uv.h>

#include <string>
#include <unordered_set>

#include "transport.h"  // Limits, RequestHandler

namespace bf::http_server {

class Connection;

class Server {
 public:
  // Serve connections routed by `handler`, applying `limits` to each. Both must
  // outlive the Server. Does no I/O until Listen().
  Server(uv_loop_t* loop, RequestHandler& handler, const Limits& limits);
  // Do not destroy a Server while the loop still polls its handles. Call
  // Shutdown() (or Close()) on the loop thread and drain with uv_run before
  // destroying the Server or the loop. The destructor still calls Close() as a
  // last resort for the listening handle only.
  ~Server();

  Server(const Server&) = delete;
  Server& operator=(const Server&) = delete;
  Server(Server&&) = delete;
  Server& operator=(Server&&) = delete;

  // Bind `host:port` and start listening. Returns 0 on success or a libuv error
  // code. Pass port 0 to let the OS choose a free port (see BoundPort).
  int Listen(const std::string& host, int port);

  // The actually-bound TCP port, or -1 if unavailable. Valid after a successful
  // Listen; useful when binding to port 0.
  int BoundPort() const;

  // Stop accepting new connections by closing the listening socket. MUST be
  // called on the loop thread (e.g. from a uv_async callback). Idempotent.
  // In-flight connections are untouched — use Shutdown() for process teardown
  // when keep-alive peers may still be open. Prefer an explicit Close/Shutdown
  // on the loop thread before destroying the loop (see ~Server).
  void Close();

  // Stop accepting and close every live Connection through Connection::Close
  // (StartClose), so self_ resets and IsAlive() goes false. MUST be called on
  // the loop thread. Idempotent; safe if live_ mutates as closes complete.
  // Prefer this over Close() + uv_walk(uv_close(..., nullptr)) for process
  // teardown — walking Connection handles with a null close callback bypasses
  // StartClose and can leave self_ alive.
  void Shutdown();

 private:
  static void OnNewConnection(uv_stream_t* server, int status);

  uv_loop_t* loop_;
  RequestHandler& handler_;
  Limits limits_;
  uv_tcp_t handle_{};
  // Live Connection pointers (loop thread only). Enforces Limits::max_connections
  // via size(); erased from SetOnClosed once both handles finish closing.
  std::unordered_set<Connection*> live_;
};

}  // namespace bf::http_server
