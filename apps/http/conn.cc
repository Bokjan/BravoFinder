// conn.cc — the per-connection HTTP/1.1 state machine and the safety hardening
// that llhttp (a pure parser) does not do. The checklist implemented here:
//
//   * request assembly: accumulate url / headers / body across llhttp callbacks
//     and dispatch on message-complete;
//   * keep-alive: honor llhttp_should_keep_alive, and fully reset the parser and
//     the per-request buffers before the next request so nothing leaks across;
//   * response framing: hand-written status line + Content-Type / Content-Length
//     / Connection / Date headers + body (we only ever send Content-Length);
//   * limits: caps on total header bytes, header count, and body size (413) to
//     bound memory;
//   * timeouts: one idle timer covers header-read, body-read, and idle
//     keep-alive, closing slow/stalled connections (slowloris);
//   * chunked rejection: any Transfer-Encoding request is refused (400) and the
//     connection closed rather than parsed, so a smuggled body cannot desync us;
//   * one request in flight: after dispatch we ignore further input bytes until
//     the response is written (no pipelining), but still react to a disconnect.
//
// Everything here runs on the libuv loop thread. The heavy query itself is
// offloaded (work.cc); only the response write comes back here.

#include "conn.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <ctime>
#include <string>
#include <utility>

#include "handlers.h"
#include "router.h"

namespace bf::http {

namespace {

// Fixed header hardening caps (body size / timeout are CLI-tunable via Limits).
constexpr size_t kMaxHeaderBytes = 32 * 1024;
constexpr int kMaxHeaderCount = 100;
// The request line's URL (path + query) is not a header, so it is capped
// separately; 8 KiB is generous for any real path + query string.
constexpr size_t kMaxUrlBytes = 8 * 1024;

const char* ReasonPhrase(int status) {
  switch (status) {
    case 200:
      return "OK";
    case 400:
      return "Bad Request";
    case 404:
      return "Not Found";
    case 408:
      return "Request Timeout";
    case 413:
      return "Payload Too Large";
    case 414:
      return "URI Too Long";
    case 422:
      return "Unprocessable Entity";
    case 431:
      return "Request Header Fields Too Large";
    case 500:
      return "Internal Server Error";
    case 503:
      return "Service Unavailable";
    default:
      // An unmapped status is unexpected (every status we emit is listed above);
      // a neutral phrase avoids a misleading "200 OK" style status line.
      return "Error";
  }
}

// RFC 1123 date for the Date header, e.g. "Sun, 06 Nov 1994 08:49:37 GMT".
std::string HttpDate() {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &now);
#else
  gmtime_r(&now, &tm);
#endif
  char buf[64];
  const size_t n = std::strftime(buf, sizeof(buf), "%a, %d %b %Y %H:%M:%S GMT", &tm);
  return std::string(buf, n);
}

// One full HTTP response with a JSON body. A non-zero elapsed_ms adds an
// X-Elapsed-Ms header carrying the query's compute cost in milliseconds.
std::string BuildResponse(int status, const std::string& body, bool keep_alive,
                          uint32_t elapsed_ms = 0) {
  std::string out;
  out.reserve(body.size() + 160);
  out += "HTTP/1.1 ";
  out += std::to_string(status);
  out += ' ';
  out += ReasonPhrase(status);
  out += "\r\nContent-Type: application/json\r\nContent-Length: ";
  out += std::to_string(body.size());
  out += "\r\nConnection: ";
  out += keep_alive ? "keep-alive" : "close";
  out += "\r\nDate: ";
  out += HttpDate();
  if (elapsed_ms > 0) {
    out += "\r\nX-Elapsed-Ms: ";
    out += std::to_string(elapsed_ms);
  }
  out += "\r\n\r\n";
  out += body;
  return out;
}

// A pending write: keeps the response bytes and a strong connection reference
// alive until libuv finishes the write.
struct WriteReq {
  uv_write_t req{};
  std::string payload;
  std::shared_ptr<Connection> conn;
  bool keep_alive = false;
};

}  // namespace

Connection::Connection(Router& router, const Limits& limits) : router_(router), limits_(limits) {}

Connection::~Connection() = default;

std::shared_ptr<Connection> Connection::Create(uv_loop_t* loop, Router& router,
                                               const Limits& limits) {
  auto conn = std::shared_ptr<Connection>(new Connection(router, limits));
  // Strong self-reference: the object outlives the local shared_ptr and every
  // libuv callback until both handles finish closing.
  conn->self_ = conn;
  uv_tcp_init(loop, &conn->handle_);
  conn->handle_.data = conn.get();
  uv_timer_init(loop, &conn->timer_);
  conn->timer_.data = conn.get();
  conn->open_handles_ = 2;
  SetupParser(*conn);
  return conn;
}

void Connection::SetupParser(Connection& conn) {
  llhttp_settings_init(&conn.settings_);
  conn.settings_.on_message_begin = OnMessageBegin;
  conn.settings_.on_url = OnUrl;
  conn.settings_.on_header_field = OnHeaderField;
  conn.settings_.on_header_value = OnHeaderValue;
  conn.settings_.on_headers_complete = OnHeadersComplete;
  conn.settings_.on_body = OnBody;
  conn.settings_.on_message_complete = OnMessageComplete;
  llhttp_init(&conn.parser_, HTTP_REQUEST, &conn.settings_);
  conn.parser_.data = &conn;
}

void Connection::Start() {
  RestartTimer();
  const int r = uv_read_start(stream(), OnAlloc, OnRead);
  if (r != 0) {
    StartClose();
  }
}

void Connection::Close() { StartClose(); }

void Connection::RestartTimer() {
  // One-shot idle timer; re-armed on every byte of progress.
  uv_timer_start(&timer_, OnTimeout, limits_.io_timeout_ms, 0);
}

void Connection::StartClose() {
  if (closing_) {
    return;
  }
  closing_ = true;
  uv_timer_stop(&timer_);
  // Each of the two handles (timer, tcp) must drop open_handles_ exactly once.
  // When a handle is already closing (shut down by an earlier EOF/error path),
  // uv_close cannot be called again and OnHandleClosed would never fire, so the
  // count is decremented here directly instead.
  if (!uv_is_closing(reinterpret_cast<uv_handle_t*>(&timer_))) {
    uv_close(reinterpret_cast<uv_handle_t*>(&timer_), OnHandleClosed);
  } else {
    --open_handles_;
  }
  if (!uv_is_closing(reinterpret_cast<uv_handle_t*>(&handle_))) {
    uv_close(reinterpret_cast<uv_handle_t*>(&handle_), OnHandleClosed);
  } else {
    --open_handles_;
  }
  if (open_handles_ == 0) {
    self_.reset();  // may delete this; nothing may follow
  }
}

void Connection::ResetForNextRequest() {
  llhttp_reset(&parser_);
  url_.clear();
  cur_field_.clear();
  cur_value_.clear();
  reading_value_ = false;
  header_bytes_ = 0;
  header_count_ = 0;
  saw_transfer_encoding_ = false;
  body_.clear();
  request_ready_ = false;
  keep_alive_ = false;
  awaiting_response_ = false;
  pipelined_ = false;
  reject_status_ = 0;
  reject_message_.clear();
  RestartTimer();
}

void Connection::OnAlloc(uv_handle_t* h, size_t /*suggested*/, uv_buf_t* buf) {
  auto* conn = static_cast<Connection*>(h->data);
  buf->base = conn->read_buf_;
  buf->len = sizeof(conn->read_buf_);
}

void Connection::OnRead(uv_stream_t* s, ssize_t nread, const uv_buf_t* buf) {
  auto* conn = static_cast<Connection*>(s->data);
  if (nread < 0) {
    // EOF or a read error: the client is gone. Any in-flight work item still
    // holds a reference, so its completion callback will find IsAlive() false
    // and drop the response.
    conn->StartClose();
    return;
  }
  if (nread == 0) {
    return;  // no data available yet (EAGAIN-equivalent)
  }
  conn->RestartTimer();
  if (conn->awaiting_response_) {
    // A request is already dispatched; we do not pipeline, so ignore any bytes
    // that arrive before we have answered it.
    return;
  }
  const llhttp_errno_t err = llhttp_execute(&conn->parser_, buf->base, static_cast<size_t>(nread));
  if (conn->reject_status_ != 0) {
    // A hardening limit tripped (body/header cap, chunked): answer and close.
    conn->awaiting_response_ = true;
    conn->WriteResponse(conn->reject_status_, bf::service::JsonError(conn->reject_message_), false);
    return;
  }
  if (conn->request_ready_) {
    // We do not pipeline. If bytes remained in this buffer after the request
    // completed -- a fully-parsed second request (llhttp does not honor the
    // message-complete pause when the trailing bytes form a valid request, so
    // OnMessageBegin catches it) or trailing bytes llhttp paused before -- we
    // cannot process them, so answer this request and close rather than silently
    // dropping them (the next uv_read would overwrite read_buf_ and lose them).
    // The client sees Connection: close and reissues on a fresh connection.
    const char* stop = llhttp_get_error_pos(&conn->parser_);
    const size_t consumed =
        stop != nullptr ? static_cast<size_t>(stop - buf->base) : static_cast<size_t>(nread);
    if (conn->pipelined_ || consumed < static_cast<size_t>(nread)) {
      conn->keep_alive_ = false;
    }
    conn->awaiting_response_ = true;
    conn->Dispatch();
    return;
  }
  if (err != HPE_OK && err != HPE_PAUSED) {
    conn->awaiting_response_ = true;
    conn->WriteResponse(400, bf::service::JsonError("malformed HTTP request"), false);
    return;
  }
  // Otherwise the request is still arriving; keep reading.
}

void Connection::OnTimeout(uv_timer_t* t) {
  auto* conn = static_cast<Connection*>(t->data);
  // A slow or idle connection (slowloris, or an idle keep-alive): close it.
  conn->StartClose();
}

int Connection::OnMessageBegin(llhttp_t* p) {
  auto* conn = static_cast<Connection*>(p->data);
  if (conn->request_ready_) {
    // A second request is starting in the same buffer (llhttp kept parsing past
    // the message-complete pause because the trailing bytes are a valid request).
    // We do not pipeline: stop here -- before this request's on_url, so url_ stays
    // the first request's -- and flag it so OnRead answers the first, then closes.
    conn->pipelined_ = true;
    return -1;
  }
  return 0;
}

int Connection::OnUrl(llhttp_t* p, const char* at, size_t len) {
  auto* conn = static_cast<Connection*>(p->data);
  conn->url_.append(at, len);
  // Cap the request-line URL incrementally: without this a multi-megabyte
  // "GET /AAAA..." grows url_ unbounded before the message even completes.
  if (conn->url_.size() > kMaxUrlBytes) {
    conn->reject_status_ = 414;
    conn->reject_message_ = "request URI too long";
    return -1;
  }
  return 0;
}

// Running header size = finalized pairs + the field/value currently arriving.
// Enforced on every chunk so a single oversized field or value trips the cap
// before it can grow unbounded -- llhttp only signals a completed pair (via
// FinishHeaderPair), not each chunk, so the per-pair check alone lets one giant
// value balloon memory before it is ever counted.
bool Connection::HeaderBudgetExceeded() const {
  return header_bytes_ + cur_field_.size() + cur_value_.size() > kMaxHeaderBytes;
}

void Connection::FinishHeaderPair() {
  if (cur_field_.empty()) {
    return;
  }
  ++header_count_;
  header_bytes_ += cur_field_.size() + cur_value_.size();
  if (header_bytes_ > kMaxHeaderBytes || header_count_ > kMaxHeaderCount) {
    reject_status_ = 431;
    reject_message_ = "request headers too large";
  }
  // We only support Content-Length bodies. Any Transfer-Encoding request is
  // refused rather than parsed, so a chunked/smuggled body cannot desync us.
  std::string lower = cur_field_;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lower == "transfer-encoding") {
    saw_transfer_encoding_ = true;
  }
  cur_field_.clear();
  cur_value_.clear();
  reading_value_ = false;
}

int Connection::OnHeaderField(llhttp_t* p, const char* at, size_t len) {
  auto* conn = static_cast<Connection*>(p->data);
  if (conn->reading_value_) {
    conn->FinishHeaderPair();  // a new field means the previous pair is complete
  }
  conn->cur_field_.append(at, len);
  if (conn->reject_status_ == 0 && conn->HeaderBudgetExceeded()) {
    conn->reject_status_ = 431;
    conn->reject_message_ = "request headers too large";
  }
  return conn->reject_status_ != 0 ? -1 : 0;
}

int Connection::OnHeaderValue(llhttp_t* p, const char* at, size_t len) {
  auto* conn = static_cast<Connection*>(p->data);
  conn->reading_value_ = true;
  conn->cur_value_.append(at, len);
  if (conn->reject_status_ == 0 && conn->HeaderBudgetExceeded()) {
    conn->reject_status_ = 431;
    conn->reject_message_ = "request headers too large";
  }
  return conn->reject_status_ != 0 ? -1 : 0;
}

int Connection::OnHeadersComplete(llhttp_t* p) {
  auto* conn = static_cast<Connection*>(p->data);
  conn->FinishHeaderPair();  // finalize the last header pair
  if (conn->reject_status_ != 0) {
    return -1;
  }
  if (conn->saw_transfer_encoding_) {
    conn->reject_status_ = 400;
    conn->reject_message_ = "chunked transfer-encoding is not supported";
    return -1;
  }
  return 0;
}

int Connection::OnBody(llhttp_t* p, const char* at, size_t len) {
  auto* conn = static_cast<Connection*>(p->data);
  if (conn->body_.size() + len > conn->limits_.max_body_bytes) {
    conn->reject_status_ = 413;
    conn->reject_message_ = "request body exceeds the configured limit";
    return -1;
  }
  conn->body_.append(at, len);
  return 0;
}

int Connection::OnMessageComplete(llhttp_t* p) {
  auto* conn = static_cast<Connection*>(p->data);
  conn->keep_alive_ = llhttp_should_keep_alive(p) != 0;
  conn->request_ready_ = true;
  // Stop consuming further bytes from this buffer: one request in flight, no
  // pipelining. llhttp_execute then returns HPE_PAUSED.
  llhttp_pause(p);
  return 0;
}

void Connection::Dispatch() {
  HttpRequest req;
  req.method = llhttp_method_name(static_cast<llhttp_method_t>(llhttp_get_method(&parser_)));
  const size_t q = url_.find('?');
  if (q == std::string::npos) {
    req.path = url_;
  } else {
    req.path = url_.substr(0, q);
    req.query = url_.substr(q + 1);
  }
  req.body = std::move(body_);
  req.keep_alive = keep_alive_;
  router_.Handle(shared_from_this(), req);
}

void Connection::WriteResponse(int status, const std::string& body, bool keep_alive,
                               uint32_t elapsed_ms) {
  if (closing_) {
    return;
  }
  // Own the write request through the uv_write handoff: if BuildResponse throws
  // (or uv_write fails) the unique_ptr frees it; on a successful queue libuv owns
  // it and OnWriteDone deletes it, so release() the pointer there.
  auto wr = std::make_unique<WriteReq>();
  wr->payload = BuildResponse(status, body, keep_alive, elapsed_ms);
  wr->conn = shared_from_this();
  wr->keep_alive = keep_alive;
  wr->req.data = wr.get();
  uv_buf_t b = uv_buf_init(wr->payload.data(), static_cast<unsigned>(wr->payload.size()));
  const int r = uv_write(&wr->req, stream(), &b, 1, OnWriteDone);
  if (r != 0) {
    StartClose();
    return;  // wr is freed as it goes out of scope
  }
  wr.release();  // libuv owns it now; freed in OnWriteDone
}

void Connection::OnWriteDone(uv_write_t* req, int status) {
  auto* wr = static_cast<WriteReq*>(req->data);
  std::shared_ptr<Connection> conn = std::move(wr->conn);
  const bool keep_alive = wr->keep_alive;
  delete wr;
  if (conn->closing_) {
    return;
  }
  if (status != 0 || !keep_alive) {
    conn->StartClose();
  } else {
    conn->ResetForNextRequest();
  }
}

void Connection::OnHandleClosed(uv_handle_t* h) {
  auto* conn = static_cast<Connection*>(h->data);
  if (--conn->open_handles_ == 0) {
    conn->self_.reset();  // may delete conn; nothing may follow
  }
}

}  // namespace bf::http
