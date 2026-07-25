// work.h — offload a unit of work to the libuv worker threadpool for the shared
// HTTP core.
//
// A heavy computation (e.g. a 10-30 ms route search) must never run on the loop
// thread. QueueWork hands a transport-neutral `std::function<WorkResult()>` to
// uv_queue_work: it runs on a worker thread, and the completion callback returns
// to the loop thread to write the response. The consumer builds the closure --
// capturing whatever it needs (registry, handler, parsed args, cycle) -- so the
// core stays free of any query/JSON knowledge.
//
// Liveness guard (the headline concurrency hazard): the work item holds a strong
// std::shared_ptr to the Connection, so the connection object and its libuv
// handle survive a client disconnect that happens while the worker runs. Back on
// the loop thread, the completion callback checks Connection::IsAlive() and
// drops the response if the client has gone. The worker itself never touches the
// connection (libuv handles are not thread-safe) -- only the closure and its
// captured state.

#pragma once

#include <uv.h>

#include <atomic>
#include <functional>
#include <memory>

#include "transport.h"  // WorkResult

namespace bf::http_server {

class Connection;

// Offload one unit of work. On a worker thread: run `work` (wrapped in a
// try/catch that turns any escaping exception into a 500). On completion (loop
// thread): write the response via `conn` if it is still alive, using
// `keep_alive` for the connection disposition. `inflight` counts in-flight work
// items: incremented once the item is queued and decremented in the completion
// callback, so the caller can shed load (503) before the queue grows without
// bound. All accesses are on the loop thread; the counter is atomic only as a
// defensive convention.
void QueueWork(std::shared_ptr<Connection> conn, uv_loop_t* loop, std::function<WorkResult()> work,
               bool keep_alive, std::atomic<int>& inflight);

}  // namespace bf::http_server
