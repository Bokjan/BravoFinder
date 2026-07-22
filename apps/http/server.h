// server.h — the TCP listener + accept loop for bf-http.
//
// Server owns the listening uv_tcp_t and turns each accepted socket into a
// Connection. It is extracted from main.cc so tests can start a real listener
// on a loopback port (Listen with port 0, then BoundPort) and drive it over an
// actual socket. All methods run on the listener's loop thread.

#pragma once

#include <uv.h>

#include <string>

#include "conn.h"  // Limits

namespace bf::http {

class Router;

class Server {
 public:
  // Serve connections routed by `router`, applying `limits` to each. Both must
  // outlive the Server. Does no I/O until Listen().
  Server(uv_loop_t* loop, Router& router, const Limits& limits);

  // Bind `host:port` and start listening. Returns 0 on success or a libuv error
  // code. Pass port 0 to let the OS choose a free port (see BoundPort).
  int Listen(const std::string& host, int port);

  // The actually-bound TCP port, or -1 if unavailable. Valid after a successful
  // Listen; useful when binding to port 0.
  int BoundPort() const;

  // Stop accepting new connections by closing the listening socket. MUST be
  // called on the loop thread (e.g. from a uv_async callback). Idempotent.
  // In-flight connections and queued work are untouched and drain on their own;
  // once they finish, a loop running UV_RUN_DEFAULT has no active handles left
  // and returns — enabling a graceful shutdown with no fixed-delay settle.
  void Close();

 private:
  static void OnNewConnection(uv_stream_t* server, int status);

  uv_loop_t* loop_;
  Router& router_;
  Limits limits_;
  uv_tcp_t handle_{};
};

}  // namespace bf::http
