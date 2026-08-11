// SPDX-License-Identifier: MIT
// conn.h — one TCP connection's HTTP/1.1 state machine for the shared HTTP core.
//
// A Connection owns a libuv TCP handle, an llhttp parser, and the per-request
// accumulation buffers. It runs entirely on the libuv loop thread: it reads
// bytes, feeds them to llhttp, assembles a complete request, and hands it to the
// RequestHandler. The handler either answers inline (probes, cycles, errors) or
// offloads the computation to the threadpool (work.h); either way the response
// is written back here via WriteResponse (or streamed via BeginStream /
// WriteEvent), still on the loop thread.
//
// Lifetime: a Connection is heap-allocated and held by a std::shared_ptr. It
// keeps a strong self-reference (self_) so it stays alive across libuv callbacks
// even with no other owner; an offloaded work item holds an additional strong
// reference so the object (and its handle) survive a client disconnect that
// happens mid-computation. self_ is released only when the handle finishes
// closing, so the object is destroyed once the last in-flight reference drops.
// See work.cc for the liveness guard that discards a response whose connection
// closed while the worker ran.
//
// llhttp does parsing only; every HTTP/1.1 semantic and safety concern (body /
// header limits, chunked rejection, keep-alive reset, timeouts, response
// framing) is handled here -- see the hardening checklist in the .cc.

#pragma once

#include <uv.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "http_status.h"  // kStatusNone
#include "llhttp.h"
#include "transport.h"  // HttpRequest, Limits, Headers, RequestHandler

namespace bf::http_server {

// Post-write behavior for a queued write: close the connection, reset it for the
// next keep-alive request, or (an open stream) leave it open for more events.
enum class WriteMode { kClose, kKeepAlive, kStream };

class Connection : public std::enable_shared_from_this<Connection> {
  // Passkey so make_shared can reach the constructor while external code still
  // cannot: the constructor is public (make_shared needs that) but requires a
  // Passkey token that only Connection can mint. This keeps Create() as the sole
  // construction path without a bare `new`.
  struct Passkey {
    explicit Passkey() = default;
  };

 public:
  // Create a connection on `loop`, initialize its TCP handle, and arm the
  // idle timer. The caller then uv_accept()s into tcp() and calls Start().
  // `on_closed` (optional) runs on the loop thread once both handles have
  // finished closing, just before the self-reference is dropped — used by
  // Server to track live connection count against Limits::max_connections.
  static std::shared_ptr<Connection> Create(uv_loop_t* loop, RequestHandler& handler,
                                            const Limits& limits,
                                            std::function<void()> on_closed = {});

  // Public only so make_shared can call it; the Passkey makes it effectively
  // private (only Create() can construct one). Do not call directly.
  Connection(Passkey, RequestHandler& handler, const Limits& limits,
             std::function<void()> on_closed);

  ~Connection();

  Connection(const Connection&) = delete;
  Connection& operator=(const Connection&) = delete;

  // The TCP stream handle, for uv_accept.
  uv_stream_t* stream() { return reinterpret_cast<uv_stream_t*>(&handle_); }

  // The loop this connection runs on (for offloading work).
  uv_loop_t* loop() { return handle_.loop; }

  // Begin reading requests and start the idle timer.
  void Start();

  // Close the connection (e.g. the acceptor could not uv_accept into it).
  // Idempotent; the object is freed once its handles finish closing.
  void Close();

  // Optional loop-thread callback invoked once both handles have finished
  // closing (just before the self-reference drops). Server uses this to track
  // live connection count; set only after a successful accept + count increment
  // so a failed accept cannot decrement a count that was never raised.
  void SetOnClosed(std::function<void()> on_closed) { on_closed_ = std::move(on_closed); }

  // True until close has been initiated. Checked on the loop thread by an
  // offloaded work item's completion callback before it writes: a false result
  // means the client went away while the worker ran, so the response is dropped.
  bool IsAlive() const { return !closing_; }

  // Write one complete HTTP response (status line + headers + body) and, when
  // keep_alive is set and the write succeeds, reset for the next request;
  // otherwise close the connection after the write drains. content_type sets the
  // Content-Type header; extra_headers are appended verbatim after the framing
  // headers. A non-zero elapsed_ms adds an X-Elapsed-Ms header. Loop thread only.
  void WriteResponse(int status, const std::string& body, bool keep_alive,
                     const std::string& content_type, const Headers& extra_headers,
                     uint32_t elapsed_ms = 0);

  // Convenience overload for the common JSON response: Content-Type
  // application/json, no extra headers.
  void WriteResponse(int status, const std::string& body, bool keep_alive, uint32_t elapsed_ms = 0);

  // Begin an open, close-delimited response stream: write the status line +
  // headers with Connection: keep-alive and NO Content-Length, then stream
  // chunks via WriteEvent until the client disconnects or the idle timer fires.
  // Used for a text/event-stream GET (SSE). Once streaming, OnWriteDone neither
  // resets for a next request nor closes on write completion. Loop thread only.
  void BeginStream(int status, const std::string& content_type, const Headers& extra_headers);

  // Write one raw chunk to an open stream (an SSE "data: ...\n\n" event or a
  // ": keepalive\n\n" comment). No-op unless a stream is open. Loop thread only.
  void WriteEvent(const std::string& chunk);

 private:
  // Begin closing the connection (idempotent). Stops the timer and closes the
  // handles; the object is freed once every handle's close callback has run and
  // no work item still references it.
  void StartClose();

  // Reset parser + per-request buffers for the next keep-alive request and
  // restart the idle timer.
  void ResetForNextRequest();

  // (Re)arm the one-shot idle timer to the configured io-timeout.
  void RestartTimer();

  // Dispatch the assembled request to the RequestHandler (loop thread).
  void Dispatch();

  // Low-level write of an already-built payload. `mode` decides post-write
  // behavior: keep-alive reset, close, or (streaming) leave the connection open.
  // The payload is the raw wire frame (opaque bytes), held alive across the
  // async uv_write in WriteReq::payload.
  void WriteRaw(std::vector<uint8_t> payload, WriteMode mode);

  // llhttp settings wiring + the static trampolines it calls back into.
  static void SetupParser(Connection& conn);
  static int OnMessageBegin(llhttp_t* p);
  static int OnUrl(llhttp_t* p, const char* at, size_t len);
  static int OnHeaderField(llhttp_t* p, const char* at, size_t len);
  static int OnHeaderValue(llhttp_t* p, const char* at, size_t len);
  static int OnHeadersComplete(llhttp_t* p);
  static int OnBody(llhttp_t* p, const char* at, size_t len);
  static int OnMessageComplete(llhttp_t* p);

  // libuv trampolines.
  static void OnAlloc(uv_handle_t* h, size_t suggested, uv_buf_t* buf);
  static void OnRead(uv_stream_t* s, ssize_t nread, const uv_buf_t* buf);
  static void OnTimeout(uv_timer_t* t);
  static void OnWriteDone(uv_write_t* req, int status);
  static void OnHandleClosed(uv_handle_t* h);

  // Record the just-finished header field/value pair: enforce the header caps,
  // note a Transfer-Encoding header (which we reject), and store the pair so the
  // handler can read request headers (e.g. Accept).
  void FinishHeaderPair();

  // Whether the accumulated headers (finalized pairs + the field/value currently
  // arriving) have exceeded the byte cap. Checked on every chunk so an oversized
  // single field or value is rejected before it can grow unbounded.
  bool HeaderBudgetExceeded() const;

  // True when the hard request deadline (Limits::request_timeout_ms from
  // message-begin) has elapsed. Idle keep-alive with no request in flight is
  // never overdue.
  bool RequestDeadlineExceeded() const;

  uv_tcp_t handle_{};
  uv_timer_t timer_{};
  llhttp_t parser_{};
  llhttp_settings_t settings_{};
  RequestHandler& handler_;
  Limits limits_;
  std::function<void()> on_closed_;

  // Self-reference keeping the object alive between callbacks; reset when the
  // last handle finishes closing.
  std::shared_ptr<Connection> self_;

  // How many of the two handles (tcp, timer) are still open, so the self-
  // reference is released only after both close callbacks have run.
  int open_handles_ = 0;
  bool closing_ = false;

  // Per-request accumulation.
  std::string url_;
  std::string cur_field_;
  std::string cur_value_;
  bool reading_value_ = false;
  size_t header_bytes_ = 0;
  int header_count_ = 0;
  bool saw_transfer_encoding_ = false;
  Headers headers_;  // finalized (lower-cased name, value) pairs
  // The raw request body, accumulated from llhttp's OnBody callbacks as opaque
  // bytes (the transport is payload-neutral; it never interprets the body as
  // text). Converted to std::string at Dispatch only because the consumer
  // contract (HttpRequest::body) is JSON text.
  std::vector<uint8_t> body_;
  bool request_ready_ = false;
  bool keep_alive_ = false;
  bool awaiting_response_ = false;  // request dispatched; further input forces close-after-reply
  bool force_close_after_response_ = false;  // pipelined bytes while awaiting; do not keep-alive
  bool pipelined_ = false;           // a second request began in the same buffer; close after reply
  bool streaming_ = false;           // an open stream is active; writes leave the connection open
  int reject_status_ = kStatusNone;  // non-zero => a hardening limit tripped; response + close
  std::string reject_message_;       // human-readable reason paired with reject_status_
  // uv_now(loop) at OnMessageBegin; 0 means no request is being assembled.
  uint64_t request_started_at_ms_ = 0;

  // A fixed read buffer; reads are one-at-a-time per connection on the loop
  // thread, so a single owned buffer suffices (no per-read allocation).
  char read_buf_[64 * 1024];
};

}  // namespace bf::http_server
