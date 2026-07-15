// conn.h — one TCP connection's HTTP/1.1 state machine for bf-http.
//
// A Connection owns a libuv TCP handle, an llhttp parser, and the per-request
// accumulation buffers. It runs entirely on the libuv loop thread: it reads
// bytes, feeds them to llhttp, assembles a complete request, and hands it to the
// Router. The Router either answers inline (probes, cycles, errors) or offloads
// the computation to the threadpool (work.h); either way the response is written
// back here via WriteResponse, still on the loop thread.
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
#include <memory>
#include <string>

#include "llhttp.h"

namespace bf::http {

class Router;

// Transport limits/timeouts. Body size and the idle timeout are CLI-tunable;
// the header caps are fixed hardening constants (see conn.cc).
struct Limits {
  size_t max_body_bytes = 1u << 20;  // 1 MiB request body cap (--max-body)
  uint64_t io_timeout_ms = 30'000;   // header/body read + idle keep-alive (--io-timeout)
};

// A fully-parsed HTTP request handed to the Router. method/path are what routing
// keys off; query carries the raw string after '?' (for ?cycle=); body is the
// raw request body (may be empty).
struct HttpRequest {
  std::string method;
  std::string path;
  std::string query;
  std::string body;
  bool keep_alive = false;
};

class Connection : public std::enable_shared_from_this<Connection> {
 public:
  // Create a connection on `loop`, initialize its TCP handle, and arm the
  // idle timer. The caller then uv_accept()s into tcp() and calls Start().
  static std::shared_ptr<Connection> Create(uv_loop_t* loop, Router& router, const Limits& limits);

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

  // True until close has been initiated. Checked on the loop thread by an
  // offloaded work item's completion callback before it writes: a false result
  // means the client went away while the worker ran, so the response is dropped.
  bool IsAlive() const { return !closing_; }

  // Write one HTTP response (status line + headers + JSON body) and, when
  // keep_alive is set and the write succeeds, reset for the next request;
  // otherwise close the connection after the write drains. Loop thread only.
  void WriteResponse(int status, const std::string& body, bool keep_alive);

 private:
  Connection(uv_loop_t* loop, Router& router, const Limits& limits);

  // Begin closing the connection (idempotent). Stops the timer and closes the
  // handles; the object is freed once every handle's close callback has run and
  // no work item still references it.
  void StartClose();

  // Reset parser + per-request buffers for the next keep-alive request and
  // restart the idle timer.
  void ResetForNextRequest();

  // (Re)arm the one-shot idle timer to the configured io-timeout.
  void RestartTimer();

  // Dispatch the assembled request to the Router (loop thread).
  void Dispatch();

  // llhttp settings wiring + the static trampolines it calls back into.
  static void SetupParser(Connection& conn);
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

  // Record the just-finished header field/value pair: enforce the header caps
  // and note a Transfer-Encoding header (which we reject).
  void FinishHeaderPair();

  uv_tcp_t handle_{};
  uv_timer_t timer_{};
  llhttp_t parser_{};
  llhttp_settings_t settings_{};
  Router& router_;
  Limits limits_;

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
  std::string body_;
  bool request_ready_ = false;
  bool keep_alive_ = false;
  bool awaiting_response_ = false;  // request dispatched; ignore further input bytes
  int reject_status_ = 0;           // non-zero => a hardening limit tripped; response + close
  std::string reject_message_;      // human-readable reason paired with reject_status_

  // A fixed read buffer; reads are one-at-a-time per connection on the loop
  // thread, so a single owned buffer suffices (no per-read allocation).
  char read_buf_[64 * 1024];
};

}  // namespace bf::http
