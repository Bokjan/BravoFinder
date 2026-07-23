// router.h — maps a parsed HTTP request to a bf::service handler for bf-http.
//
// The Router is the RequestHandler for the shared HTTP core: it runs on the
// libuv loop thread, answers the cheap endpoints inline (liveness probe, cycle
// list, not-found / bad-request errors) and offloads everything that touches a
// database -- the query endpoints and the readiness probe -- to the threadpool
// via http_server::QueueWork, so the 10-30 ms route computation never blocks the
// loop. Endpoint -> handler wiring, ?cycle= parsing, and status-code selection
// all live here; the transport (connection state machine, listener, offload
// plumbing) lives in the shared http_server/ library.

#pragma once

#include <uv.h>

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>

#include "handlers.h"
#include "registry.h"
#include "transport.h"  // http_server::RequestHandler, HttpRequest, Connection (fwd)

namespace bf::http {

class Router : public http_server::RequestHandler {
 public:
  // Serve queries from `registry`; offload work onto `loop`'s threadpool. Both
  // must outlive the Router.
  Router(bf::service::NavDatabaseRegistry& registry, uv_loop_t* loop);

  // Route one fully-parsed request (loop thread): write an inline response, or
  // queue an offloaded query, using `conn` for the reply.
  void Handle(std::shared_ptr<http_server::Connection> conn,
              const http_server::HttpRequest& req) override;

 private:
  // Serialize the available AIRAC cycles, newest first, as {"cycles":[...]}.
  std::string SerializeCycles() const;

  bf::service::NavDatabaseRegistry& registry_;
  uv_loop_t* loop_;
  // POST path (e.g. "/v1/routes") -> the shared handler that serves it.
  std::unordered_map<std::string, bf::service::QueryHandler> routes_;
  // In-flight offloaded work items. Incremented when queued, decremented on
  // completion (both on the loop thread); a request that would exceed the cap is
  // shed with 503 before it can grow the threadpool queue without bound.
  std::atomic<int> inflight_{0};
};

}  // namespace bf::http
