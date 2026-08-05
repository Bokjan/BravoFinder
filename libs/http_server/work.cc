// SPDX-License-Identifier: MIT
// work.cc — the uv_queue_work offload and its liveness-guarded completion.

#include "work.h"

#include <exception>
#include <format>
#include <memory>
#include <string>
#include <utility>

#include "conn.h"

namespace bf::http_server {

namespace {

// One offloaded unit of work. Lives from QueueWork until the completion callback
// runs. The worker runs only `work` and writes `result`; it must not touch
// `conn` (libuv handles are not thread-safe). `conn` keeps the connection alive
// so the completion callback can write back (or find it gone).
struct WorkRequest {
  uv_work_t req{};
  std::shared_ptr<Connection> conn;
  std::function<WorkResult()> work;
  bool keep_alive = false;
  std::atomic<int>* inflight = nullptr;  // decremented once, in the completion cb
  WorkResult result{};
};

// Worker thread: run the closure. Never touches conn.
void OnWork(uv_work_t* req) {
  auto* w = static_cast<WorkRequest*>(req->data);
  try {
    w->result = w->work();
  } catch (const std::exception& e) {
    // The closure should go through Result, but never let an unexpected
    // exception cross the thread boundary: turn it into a 500.
    w->result.status = kStatusInternalServerError;
    w->result.body = JsonError(std::format("internal error: {}", e.what()));
  } catch (...) {
    w->result.status = kStatusInternalServerError;
    w->result.body = JsonError("internal error");
  }
}

// Loop thread: write the response if the connection is still alive, then free
// the work item (dropping its connection reference).
void OnAfterWork(uv_work_t* req, int status) {
  std::unique_ptr<WorkRequest> w(static_cast<WorkRequest*>(req->data));
  w->inflight->fetch_sub(1, std::memory_order_relaxed);
  if (!w->conn->IsAlive()) {
    return;  // the client left while we computed -- drop the response
  }
  if (status == 0) {
    const std::string content_type =
        w->result.content_type.empty() ? "application/json" : w->result.content_type;
    w->conn->WriteResponse(w->result.status, w->result.body, w->keep_alive, content_type,
                           w->result.extra_headers, w->result.elapsed_ms);
  } else {
    // status != 0 means the work was cancelled (we never request this). Rather
    // than leave the connection hanging until the idle timeout, close it.
    w->conn->Close();
  }
}

}  // namespace

void QueueWork(std::shared_ptr<Connection> conn, uv_loop_t* loop, std::function<WorkResult()> work,
               bool keep_alive, std::atomic<int>& inflight) {
  // Bare new/delete here is the integer half of a libuv C-callback handoff, not
  // an unmanaged allocation: uv_queue_work takes a raw uv_work_t* and, on a
  // successful queue, OnAfterWork reclaims it via unique_ptr. On the failure
  // path below (queue unavailable) nothing was queued, so we delete it directly.
  auto* w = new WorkRequest();
  w->conn = std::move(conn);
  w->work = std::move(work);
  w->keep_alive = keep_alive;
  w->inflight = &inflight;
  w->req.data = w;
  const int r = uv_queue_work(loop, &w->req, OnWork, OnAfterWork);
  if (r != 0) {
    // The threadpool queue is unavailable: answer 503 inline and clean up. No
    // increment happened, so nothing to undo.
    if (w->conn->IsAlive()) {
      w->conn->WriteResponse(kStatusServiceUnavailable, JsonError("server busy"), keep_alive);
    }
    delete w;
    return;
  }
  inflight.fetch_add(1, std::memory_order_relaxed);
}

}  // namespace bf::http_server
