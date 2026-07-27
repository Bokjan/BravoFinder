// SPDX-License-Identifier: MIT
// conn.cc — the per-connection HTTP/1.1 state machine and the safety hardening
// that llhttp (a pure parser) does not do. The checklist implemented here:
//
//   * request assembly: accumulate url / headers / body across llhttp callbacks
//     and dispatch on message-complete;
//   * keep-alive: honor llhttp_should_keep_alive, and fully reset the parser and
//     the per-request buffers before the next request so nothing leaks across;
//   * response framing: hand-written status line + Content-Type / Content-Length
//     / Connection / Date headers + body, or a close-delimited open stream (SSE)
//     with no Content-Length (BeginStream / WriteEvent);
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

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "http_status.h"

namespace bf::http_server {

namespace {

// Fixed header hardening caps (body size / timeout are CLI-tunable via Limits).
constexpr size_t kMaxHeaderBytes = 32 * 1024;
constexpr int kMaxHeaderCount = 100;
// The request line's URL (path + query) is not a header, so it is capped
// separately; 8 KiB is generous for any real path + query string.
constexpr size_t kMaxUrlBytes = 8 * 1024;

const char* ReasonPhrase(int status) {
  switch (status) {
    case kStatusOk:
      return "OK";
    case kStatusAccepted:
      return "Accepted";
    case kStatusBadRequest:
      return "Bad Request";
    case kStatusNotFound:
      return "Not Found";
    case kStatusMethodNotAllowed:
      return "Method Not Allowed";
    case kStatusRequestTimeout:
      return "Request Timeout";
    case kStatusPayloadTooLarge:
      return "Payload Too Large";
    case kStatusUriTooLong:
      return "URI Too Long";
    case kStatusUnprocessableEntity:
      return "Unprocessable Entity";
    case kStatusRequestHeaderFieldsTooLarge:
      return "Request Header Fields Too Large";
    case kStatusInternalServerError:
      return "Internal Server Error";
    case kStatusServiceUnavailable:
      return "Service Unavailable";
    default:
      // An unmapped status is unexpected (every status we emit is listed above).
      // Fall back by class so a future unmapped code still yields a sane phrase
      // rather than a misleading "200 OK" style line or a bare "Error".
      if (status < kStatusSuccessMin) {
        return "Informational";
      }
      if (status < kStatusRedirectionMin) {
        return "OK";
      }
      if (status < kStatusClientErrorMin) {
        return "Redirection";
      }
      if (status < kStatusServerErrorMin) {
        return "Client Error";
      }
      return "Server Error";
  }
}

// Append raw bytes from a text fragment into the wire frame buffer. The frame
// is opaque bytes (std::vector<uint8_t>); status line, header names/values,
// and the body all get appended through here, centralizing the single
// reinterpret_cast from char* to uint8_t* for the text portions.
void AppendBytes(std::vector<uint8_t>& out, const char* s, size_t n) {
  out.insert(out.end(), reinterpret_cast<const uint8_t*>(s),
             reinterpret_cast<const uint8_t*>(s + n));
}
void AppendBytes(std::vector<uint8_t>& out, std::string_view s) {
  AppendBytes(out, s.data(), s.size());
}

// Append `v` to `out`, dropping any CR / LF / NUL byte. Header names and values
// are concatenated into the response with raw byte ops, so a stray \r\n in a
// value would inject headers or split the response (HTTP response splitting).
// Every current caller passes trusted content (hardcoded content types, a
// hex-only session id), but the transport core must not rely on that.
void AppendHeaderSafe(std::vector<uint8_t>& out, std::string_view v) {
  for (const char c : v) {
    if (c != '\r' && c != '\n' && c != '\0') {
      out.push_back(static_cast<uint8_t>(c));
    }
  }
}

// Append a header NAME to `out`: drops CR / LF / NUL like AppendHeaderSafe and
// additionally any space or ':'. Either of those in a name would split the
// "name: value" framing below and forge a header. Header names are RFC 9110
// tokens (no space, no colon), so a well-formed name is unchanged; this only
// hardens the transport core against a future caller passing a half-trusted name.
void AppendHeaderNameSafe(std::vector<uint8_t>& out, std::string_view name) {
  for (const char c : name) {
    if (c != '\r' && c != '\n' && c != '\0' && c != ' ' && c != ':') {
      out.push_back(static_cast<uint8_t>(c));
    }
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

// Append the extra (name: value) headers verbatim after the framing headers.
void AppendExtraHeaders(std::vector<uint8_t>& out, const Headers& extra_headers) {
  for (const auto& [name, value] : extra_headers) {
    AppendBytes(out, "\r\n");
    AppendHeaderNameSafe(out, name);
    AppendBytes(out, ": ");
    AppendHeaderSafe(out, value);
  }
}

// One full HTTP response with a Content-Length body. A non-zero elapsed_ms adds
// an X-Elapsed-Ms header carrying the query's compute cost in milliseconds.
// The returned frame is opaque wire bytes (std::vector<uint8_t>); `body` is JSON
// text from the consumer, appended as bytes.
std::vector<uint8_t> BuildResponse(int status, const std::string& body, bool keep_alive,
                                   const std::string& content_type, const Headers& extra_headers,
                                   uint32_t elapsed_ms) {
  std::vector<uint8_t> out;
  out.reserve(body.size() + 200);
  AppendBytes(out, "HTTP/1.1 ");
  const std::string status_str = std::to_string(status);
  AppendBytes(out, status_str);
  AppendBytes(out, " ");
  AppendBytes(out, ReasonPhrase(status));
  AppendBytes(out, "\r\nContent-Type: ");
  AppendHeaderSafe(out, content_type);
  AppendBytes(out, "\r\nContent-Length: ");
  AppendBytes(out, std::to_string(body.size()));
  AppendBytes(out, "\r\nConnection: ");
  AppendBytes(out, keep_alive ? "keep-alive" : "close");
  AppendBytes(out, "\r\nDate: ");
  AppendBytes(out, HttpDate());
  if (elapsed_ms > 0) {
    AppendBytes(out, "\r\nX-Elapsed-Ms: ");
    AppendBytes(out, std::to_string(elapsed_ms));
  }
  AppendExtraHeaders(out, extra_headers);
  AppendBytes(out, "\r\n\r\n");
  AppendBytes(out, body);
  return out;
}

// The header block for an open, close-delimited stream: no Content-Length (the
// stream ends when the connection closes), Connection: keep-alive.
std::vector<uint8_t> BuildStreamHeader(int status, const std::string& content_type,
                                       const Headers& extra_headers) {
  std::vector<uint8_t> out;
  AppendBytes(out, "HTTP/1.1 ");
  AppendBytes(out, std::to_string(status));
  AppendBytes(out, " ");
  AppendBytes(out, ReasonPhrase(status));
  AppendBytes(out, "\r\nContent-Type: ");
  AppendHeaderSafe(out, content_type);
  AppendBytes(out, "\r\nConnection: keep-alive\r\nDate: ");
  AppendBytes(out, HttpDate());
  AppendExtraHeaders(out, extra_headers);
  AppendBytes(out, "\r\n\r\n");
  return out;
}

// A pending write: keeps the response bytes and a strong connection reference
// alive until libuv finishes the write. `mode` decides post-write behavior.
struct WriteReq {
  uv_write_t req{};
  std::vector<uint8_t> payload;
  std::shared_ptr<Connection> conn;
  WriteMode mode = WriteMode::kClose;
};

}  // namespace

std::string JsonError(const std::string& message) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("error");
  writer.String(message.c_str(), static_cast<unsigned>(message.size()));
  writer.EndObject();
  return buffer.GetString();
}

std::string HttpRequest::Header(const std::string& lower_name) const {
  for (const auto& [name, value] : headers) {
    if (name == lower_name) {
      return value;
    }
  }
  return "";
}

Connection::Connection(Passkey, RequestHandler& handler, const Limits& limits)
    : handler_(handler), limits_(limits) {}

Connection::~Connection() = default;

std::shared_ptr<Connection> Connection::Create(uv_loop_t* loop, RequestHandler& handler,
                                               const Limits& limits) {
  auto conn = std::make_shared<Connection>(Passkey{}, handler, limits);
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
  headers_.clear();
  body_.clear();
  request_ready_ = false;
  keep_alive_ = false;
  awaiting_response_ = false;
  pipelined_ = false;
  streaming_ = false;
  reject_status_ = kStatusNone;
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
  if (conn->awaiting_response_) {
    // A request is already dispatched; we do not pipeline, so ignore any bytes
    // that arrive before we have answered it -- and do NOT restart the idle
    // timer for them: otherwise a client could keep dribbling bytes to hold the
    // connection (and its shared_ptr<Connection> + in-flight WorkRequest) alive
    // indefinitely while its dispatched work is still running.
    return;
  }
  conn->RestartTimer();
  const llhttp_errno_t err = llhttp_execute(&conn->parser_, buf->base, static_cast<size_t>(nread));
  if (conn->reject_status_ != kStatusNone) {
    // A hardening limit tripped (body/header cap, chunked): answer and close.
    conn->awaiting_response_ = true;
    conn->WriteResponse(conn->reject_status_, JsonError(conn->reject_message_), false);
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
    conn->WriteResponse(kStatusBadRequest, JsonError("malformed HTTP request"), false);
    return;
  }
  // Otherwise the request is still arriving; keep reading.
}

void Connection::OnTimeout(uv_timer_t* t) {
  auto* conn = static_cast<Connection*>(t->data);
  if (conn->awaiting_response_) {
    // A request is already dispatched: an offloaded route computation is running
    // on the threadpool, or an SSE stream is open. The read phase is over, so the
    // idle timer no longer applies -- it bounds slow/idle READS and keep-alive
    // idle, NOT compute. The route computation is bounded (k is capped, the graph
    // is fixed), so the worker will finish and its completion callback writes the
    // response on the loop thread; the in-flight work holds a strong self-
    // reference, so the connection cannot leak meanwhile. Closing here would race
    // that write and silently drop a valid-but-slow response, so leave it be.
    return;
  }
  if (!conn->closing_) {
    // Slow or idle client with nothing dispatched (slowloris, or an idle keep-
    // alive): answer 408 per RFC 9110 §15.5.7 (SHOULD) so the client learns why
    // the connection dropped; the false keep_alive makes the write close after.
    conn->awaiting_response_ = true;
    conn->WriteResponse(kStatusRequestTimeout, JsonError("request timeout"), false);
    return;
  }
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
    conn->reject_status_ = kStatusUriTooLong;
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
    reject_status_ = kStatusRequestHeaderFieldsTooLarge;
    reject_message_ = "request headers too large";
  }
  // Store the header (lower-cased name) so the handler can read it -- e.g. the
  // Accept header for SSE content negotiation. We only support Content-Length
  // bodies: any Transfer-Encoding request is refused rather than parsed, so a
  // chunked/smuggled body cannot desync us.
  std::string lower = cur_field_;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lower == "transfer-encoding") {
    saw_transfer_encoding_ = true;
  }
  headers_.emplace_back(std::move(lower), cur_value_);
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
  if (conn->reject_status_ == kStatusNone && conn->HeaderBudgetExceeded()) {
    conn->reject_status_ = kStatusRequestHeaderFieldsTooLarge;
    conn->reject_message_ = "request headers too large";
  }
  return conn->reject_status_ != kStatusNone ? -1 : 0;
}

int Connection::OnHeaderValue(llhttp_t* p, const char* at, size_t len) {
  auto* conn = static_cast<Connection*>(p->data);
  conn->reading_value_ = true;
  conn->cur_value_.append(at, len);
  if (conn->reject_status_ == kStatusNone && conn->HeaderBudgetExceeded()) {
    conn->reject_status_ = kStatusRequestHeaderFieldsTooLarge;
    conn->reject_message_ = "request headers too large";
  }
  return conn->reject_status_ != kStatusNone ? -1 : 0;
}

int Connection::OnHeadersComplete(llhttp_t* p) {
  auto* conn = static_cast<Connection*>(p->data);
  conn->FinishHeaderPair();  // finalize the last header pair
  if (conn->reject_status_ != kStatusNone) {
    return -1;
  }
  if (conn->saw_transfer_encoding_) {
    conn->reject_status_ = kStatusBadRequest;
    conn->reject_message_ = "chunked transfer-encoding is not supported";
    return -1;
  }
  return 0;
}

int Connection::OnBody(llhttp_t* p, const char* at, size_t len) {
  auto* conn = static_cast<Connection*>(p->data);
  if (conn->body_.size() + len > conn->limits_.max_body_bytes) {
    conn->reject_status_ = kStatusPayloadTooLarge;
    conn->reject_message_ = "request body exceeds the configured limit";
    return -1;
  }
  conn->body_.insert(conn->body_.end(), reinterpret_cast<const uint8_t*>(at),
                     reinterpret_cast<const uint8_t*>(at + len));
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
  // The transport holds the body as opaque bytes (body_ is vector<uint8_t>);
  // the consumer contract (HttpRequest::body) is JSON text, so copy once at
  // this seam. The body is bounded by limits_.max_body_bytes.
  req.body.assign(reinterpret_cast<const char*>(body_.data()), body_.size());
  body_.clear();
  req.headers = std::move(headers_);
  req.keep_alive = keep_alive_;
  handler_.Handle(shared_from_this(), req);
}

void Connection::WriteRaw(std::vector<uint8_t> payload, WriteMode mode) {
  if (closing_) {
    return;
  }
  // Own the write request through the uv_write handoff: if uv_write fails the
  // unique_ptr frees it; on a successful queue libuv owns it and OnWriteDone
  // deletes it, so release() the pointer there.
  auto wr = std::make_unique<WriteReq>();
  wr->payload = std::move(payload);
  wr->conn = shared_from_this();
  wr->mode = mode;
  wr->req.data = wr.get();
  // uv_buf_init's base is char* (libuv) and its length arg is unsigned int
  // (uv.h), so the length is a 32-bit field regardless of the cast -- fine here
  // since every response body (JSON) is far under 4 GiB. A >4 GiB payload would
  // need splitting across multiple uv_buf_t.
  uv_buf_t b = uv_buf_init(reinterpret_cast<char*>(wr->payload.data()),
                           static_cast<unsigned>(wr->payload.size()));
  const int r = uv_write(&wr->req, stream(), &b, 1, OnWriteDone);
  if (r != 0) {
    StartClose();
    return;  // wr is freed as it goes out of scope
  }
  wr.release();  // libuv owns it now; freed in OnWriteDone
}

void Connection::WriteResponse(int status, const std::string& body, bool keep_alive,
                               const std::string& content_type, const Headers& extra_headers,
                               uint32_t elapsed_ms) {
  WriteRaw(BuildResponse(status, body, keep_alive, content_type, extra_headers, elapsed_ms),
           keep_alive ? WriteMode::kKeepAlive : WriteMode::kClose);
}

void Connection::WriteResponse(int status, const std::string& body, bool keep_alive,
                               uint32_t elapsed_ms) {
  WriteResponse(status, body, keep_alive, "application/json", {}, elapsed_ms);
}

void Connection::BeginStream(int status, const std::string& content_type,
                             const Headers& extra_headers) {
  if (closing_) {
    return;
  }
  // Mark streaming before the write so OnWriteDone leaves the connection open
  // for subsequent events rather than resetting/closing.
  streaming_ = true;
  WriteRaw(BuildStreamHeader(status, content_type, extra_headers), WriteMode::kStream);
}

void Connection::WriteEvent(const std::string& chunk) {
  if (closing_ || !streaming_) {
    return;
  }
  // SSE chunk is JSON text from the consumer; the wire payload is opaque bytes,
  // so move it into a vector here. Chunks are small ("data: ...\n\n").
  std::vector<uint8_t> payload(chunk.begin(), chunk.end());
  WriteRaw(std::move(payload), WriteMode::kStream);
}

void Connection::OnWriteDone(uv_write_t* req, int status) {
  auto* wr = static_cast<WriteReq*>(req->data);
  std::shared_ptr<Connection> conn = std::move(wr->conn);
  const WriteMode mode = wr->mode;
  // Bare delete (paired with the wr.release() in WriteRaw): libuv's uv_write_t C
  // callback hands ownership back here by raw pointer, so this is the integer
  // half of the RAII handoff, not an unmanaged allocation.
  delete wr;
  if (conn->closing_) {
    return;
  }
  if (status != 0) {
    conn->StartClose();
    return;
  }
  switch (mode) {
    case WriteMode::kStream:
      return;  // an open stream: keep the connection alive for more events
    case WriteMode::kKeepAlive:
      conn->ResetForNextRequest();
      return;
    case WriteMode::kClose:
      conn->StartClose();
      return;
  }
}

void Connection::OnHandleClosed(uv_handle_t* h) {
  auto* conn = static_cast<Connection*>(h->data);
  if (--conn->open_handles_ == 0) {
    conn->self_.reset();  // may delete conn; nothing may follow
  }
}

}  // namespace bf::http_server
