// work.cc — the uv_queue_work offload and its liveness-guarded completion.

#include "work.h"

#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "conn.h"

namespace bf::http {

namespace {

// One offloaded query. Lives from QueueQuery until the completion callback runs.
// The worker touches only registry/handler/args/cycle and writes `result`; it
// must not touch `conn` (libuv handles are not thread-safe). `conn` keeps the
// connection alive so the completion callback can write back (or find it gone).
struct WorkRequest {
  uv_work_t req{};
  std::shared_ptr<Connection> conn;
  bf::service::NavDatabaseRegistry* registry = nullptr;
  bf::service::QueryHandler handler;
  rapidjson::Document args;
  std::optional<uint32_t> cycle;
  bool keep_alive = false;
  int cycle_error_status = 400;
  bf::service::HandlerResult result{};
};

// Worker thread: resolve the database and run the handler. Never touches conn.
void OnWork(uv_work_t* req) {
  auto* w = static_cast<WorkRequest*>(req->data);
  try {
    bf::Result<const bf::NavDatabase*> db = w->registry->Get(w->cycle);
    if (!db) {
      w->result = {bf::service::JsonError(db.error().message), w->cycle_error_status};
      return;
    }
    w->result = w->handler(w->args, *db.value());
  } catch (const std::exception& e) {
    // Handlers go through Result, but never let an unexpected exception cross
    // the thread boundary: turn it into a 500.
    w->result = {bf::service::JsonError(std::string("internal error: ") + e.what()), 500};
  } catch (...) {
    w->result = {bf::service::JsonError("internal error"), 500};
  }
}

// Loop thread: write the response if the connection is still alive, then free
// the work item (dropping its connection reference).
void OnAfterWork(uv_work_t* req, int status) {
  std::unique_ptr<WorkRequest> w(static_cast<WorkRequest*>(req->data));
  // status is non-zero only on cancellation, which we never request; either way
  // a dead connection means the client left while we computed -- drop it.
  if (status == 0 && w->conn->IsAlive()) {
    w->conn->WriteResponse(w->result.status, w->result.body, w->keep_alive);
  }
}

}  // namespace

void QueueQuery(std::shared_ptr<Connection> conn, uv_loop_t* loop,
                bf::service::NavDatabaseRegistry& registry, bf::service::QueryHandler handler,
                rapidjson::Document args, std::optional<uint32_t> cycle, bool keep_alive,
                int cycle_error_status) {
  auto* w = new WorkRequest();
  w->conn = std::move(conn);
  w->registry = &registry;
  w->handler = std::move(handler);
  w->args = std::move(args);
  w->cycle = cycle;
  w->keep_alive = keep_alive;
  w->cycle_error_status = cycle_error_status;
  w->req.data = w;
  const int r = uv_queue_work(loop, &w->req, OnWork, OnAfterWork);
  if (r != 0) {
    // The threadpool queue is unavailable: answer 503 inline and clean up.
    if (w->conn->IsAlive()) {
      w->conn->WriteResponse(503, bf::service::JsonError("server busy"), keep_alive);
    }
    delete w;
  }
}

}  // namespace bf::http
