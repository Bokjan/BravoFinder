// work.h — offload a database query to the libuv worker threadpool for bf-http.
//
// The 10-30 ms route computation must never run on the loop thread. QueueQuery
// hands it to uv_queue_work: on a worker thread it resolves the database for the
// requested cycle and runs the shared handler (both read-only and thread-safe
// per NavDatabase contract B), then the completion callback returns to the loop
// thread to write the response.
//
// Liveness guard (the headline concurrency hazard): the work item holds a strong
// std::shared_ptr to the Connection, so the connection object and its libuv
// handle survive a client disconnect that happens while the worker runs. Back on
// the loop thread, the completion callback checks Connection::IsAlive() and
// drops the response if the client has gone. The worker itself never touches the
// connection (libuv handles are not thread-safe) -- only args, cycle, registry,
// and the result.

#pragma once

#include <uv.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

#include "handlers.h"
#include "rapidjson/document.h"
#include "registry.h"

namespace bf::http {

class Connection;

// Offload one query. On a worker thread: registry.Get(cycle) then handler(args);
// on completion (loop thread): write the response via `conn` if it is still
// alive. `cycle_error_status` is the status used when the cycle cannot be
// resolved (400 for query endpoints, 503 for the readiness probe). Takes
// ownership of `args`. `inflight` counts in-flight work items: incremented once
// the item is queued and decremented in the completion callback, so the caller
// can shed load (503) before the queue grows without bound. All accesses are on
// the loop thread; the counter is atomic only as a defensive convention.
void QueueQuery(std::shared_ptr<Connection> conn, uv_loop_t* loop,
                bf::service::NavDatabaseRegistry& registry, bf::service::QueryHandler handler,
                rapidjson::Document args, std::optional<uint32_t> cycle, bool keep_alive,
                int cycle_error_status, std::atomic<int>& inflight);

}  // namespace bf::http
