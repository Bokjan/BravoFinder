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
//   * limits: caps on total header bytes, header count, body size (413; also
//     early reject when Content-Length declares over the cap), and live
//     connections (Server::max_connections);
//   * timeouts: one idle timer covers header-read, body-read, and idle
//     keep-alive; a separate hard request deadline (from message-begin) catches
//     slow-drip clients that keep resetting the idle timer;
//   * chunked rejection: any Transfer-Encoding request is refused (400) and the
//     connection closed rather than parsed, so a smuggled body cannot desync us;
//   * Expect: 100-continue is refused with 417 (we never emit an interim 100);
//   * one request in flight: after dispatch we do not pipeline — further input
//     bytes are ignored and force Connection: close after the response so a
//     keep-alive client cannot desync on dropped pipelined requests.
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
#include <limits>
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
    case kStatusForbidden:
      return "Forbidden";
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
    case kStatusExpectationFailed:
      return "Expectation Failed";
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

// ASCII case-insensitive equality (HTTP header names are case-insensitive per
// RFC 9110). Shared by the reserved-framing guard and request header lookup.
bool AsciiIeq(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    const auto ca = static_cast<unsigned char>(a[i]);
    const auto cb = static_cast<unsigned char>(b[i]);
    if (std::tolower(ca) != std::tolower(cb)) {
      return false;
    }
  }
  return true;
}

// True when `name` is a framing header the core always writes itself. Extra
// headers must not override these (duplicate/conflicting framing breaks clients).
bool IsReservedFramingHeader(std::string_view name) {
  return AsciiIeq(name, "Content-Length") || AsciiIeq(name, "Content-Type") ||
         AsciiIeq(name, "Connection") || AsciiIeq(name, "Date") ||
         AsciiIeq(name, "Transfer-Encoding");
}

// Append the extra (name: value) headers after the framing headers. Reserved
// framing names are dropped so a handler cannot forge Content-Length etc.
void AppendExtraHeaders(std::vector<uint8_t>& out, const Headers& extra_headers) {
  for (const auto& [name, value] : extra_headers) {
    if (IsReservedFramingHeader(name)) {
      continue;
    }
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
                                   uint32_t elapsed_ms, size_t content_length) {
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
  AppendBytes(out, std::to_string(content_length));
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

// The host part of an authority string ("host:port", "[::1]:80") or of an
// Origin URL ("https://host/path"). Returns "" when there is no host.
std::string_view HostOf(std::string_view authority) {
  const size_t scheme = authority.find("://");
  size_t begin = scheme == std::string_view::npos ? 0 : scheme + 3;
  size_t end = authority.find_first_of("/?#", begin);
  if (end == std::string_view::npos) {
    end = authority.size();
  }
  std::string_view host_port = authority.substr(begin, end - begin);
  // Strip the port: everything after the last ':' outside IPv6 brackets, and the
  // brackets themselves so "[::1]" compares as "::1".
  if (!host_port.empty() && host_port.front() == '[') {
    const size_t close = host_port.find(']');
    if (close != std::string_view::npos) {
      return host_port.substr(1, close - 1);
    }
    return {};
  }
  const size_t colon = host_port.find(':');
  return colon == std::string_view::npos ? host_port : host_port.substr(0, colon);
}

bool OriginAllowed(const HttpRequest& req, std::string_view bind_host) {
  const std::string origin = req.Header("Origin");
  if (origin.empty()) {
    return true;  // no Origin: not a browser cross-origin request
  }
  // "Origin: null" (a sandboxed frame, a file:// page) names no host at all.
  const std::string_view host = HostOf(origin);
  if (host.empty()) {
    return false;
  }
  if (AsciiIeq(host, "localhost") || host == "127.0.0.1" || host == "::1") {
    return true;  // a same-machine page
  }
  // Otherwise the origin must be the host the request is addressed to. A browser
  // sets Origin from the page it loaded, so a same-origin request (a UI served
  // from the same server) matches its Host header, while a page on another site
  // reading 127.0.0.1 cannot produce one. Comparing against Host rather than the
  // bind address also covers a wildcard bind, where the bind address is 0.0.0.0.
  const std::string_view host_header = HostOf(req.Header("Host"));
  if (!host_header.empty() && AsciiIeq(host, host_header)) {
    return true;
  }
  // A server bound to a named LAN address may also be reached through it.
  return !bind_host.empty() && bind_host != "0.0.0.0" && bind_host != "::" &&
         AsciiIeq(host, bind_host);
}

std::string HttpRequest::Header(const std::string& name) const {
  for (const auto& [hdr_name, value] : headers) {
    if (AsciiIeq(hdr_name, name)) {
      return value;
    }
  }
  return "";
}

Connection::Connection(Passkey, RequestHandler& handler, const Limits& limits,
                       std::function<void()> on_closed)
    : handler_(handler), limits_(limits), on_closed_(std::move(on_closed)) {}

Connection::~Connection() = default;

std::shared_ptr<Connection> Connection::Create(uv_loop_t* loop, RequestHandler& handler,
                                               const Limits& limits,
                                               std::function<void()> on_closed) {
  auto conn = std::make_shared<Connection>(Passkey{}, handler, limits, std::move(on_closed));
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
  // Detect a peer that vanished without a FIN (a dropped network, a killed VM)
  // while the connection sits in keep-alive. libuv keeps the OS defaults for
  // the probe intervals; this only turns the facility on, so a dead peer is
  // reaped by the kernel instead of pinning a connection slot forever.
  uv_tcp_keepalive(&handle_, 1, 60);
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

void Connection::ShutdownThenClose() {
  if (closing_) {
    return;
  }
  // Stop reading first: nothing more of the peer's upload is wanted, and
  // uv_shutdown then completes as soon as the queued FIN is written.
  uv_read_stop(stream());
  // libuv's shutdown request is C-owned: it hands the struct back to the
  // callback, so it is allocated and released on that contract (like WriteReq).
  auto* req = new uv_shutdown_t{};
  req->data = this;
  const int r = uv_shutdown(req, stream(), OnShutdown);
  if (r != 0) {
    delete req;
    StartClose();
    return;
  }
  // The peer may never acknowledge (or may keep sending): close unconditionally
  // after the grace period so a shutdown that never completes cannot pin the
  // connection. StartClose is idempotent; the callback above is then a no-op.
  close_after_shutdown_ = true;
  uv_timer_start(&timer_, OnTimeout, limits_.io_timeout_ms, 0);
}

void Connection::OnShutdown(uv_shutdown_t* req, int /*status*/) {
  auto* conn = static_cast<Connection*>(req->data);
  delete req;
  conn->StartClose();
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
    if (on_closed_) {
      on_closed_();
      on_closed_ = nullptr;
    }
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
  force_close_after_response_ = false;
  pipelined_ = false;
  streaming_ = false;
  reject_status_ = kStatusNone;
  reject_message_.clear();
  request_started_at_ms_ = 0;
  head_request_ = false;
  close_after_shutdown_ = false;
  write_started_at_ms_ = 0;
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
    // A request is already dispatched; we do not pipeline. Drop the bytes, do
    // NOT restart the idle timer (a drip must not hold the connection + in-flight
    // WorkRequest alive), and force Connection: close after the response so the
    // client cannot keep-alive onto a desynced stream where its next request was
    // already discarded.
    conn->force_close_after_response_ = true;
    conn->keep_alive_ = false;
    return;
  }
  if (conn->RequestDeadlineExceeded()) {
    conn->awaiting_response_ = true;
    conn->WriteResponse(kStatusRequestTimeout, JsonError("request timeout"), false);
    return;
  }
  conn->RestartTimer();
  if (conn->request_started_at_ms_ == 0) {
    // First byte of a request. Stamped here as well as on message-begin because
    // llhttp's start state consumes leading CR/LF inside the parser without
    // ever signalling a message, so a client dribbling "\r\n" one byte per
    // idle timeout kept request_started_at_ms_ at 0 and never met the deadline.
    conn->request_started_at_ms_ = uv_now(conn->handle_.loop);
  }
  const llhttp_errno_t err = llhttp_execute(&conn->parser_, buf->base, static_cast<size_t>(nread));
  if (conn->reject_status_ != kStatusNone) {
    // A hardening limit tripped (body/header cap, chunked, Expect, …): answer and close.
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
      conn->force_close_after_response_ = true;
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
  if (conn->close_after_shutdown_) {
    // Half-closed and waiting for the shuffle to finish: it did not, so close.
    conn->StartClose();
    return;
  }
  if (conn->streaming_) {
    // An open SSE stream has been idle for io_timeout_ms with no WriteEvent.
    // Close it so a forgotten placeholder stream cannot hold the connection
    // (and its read buffer) forever.
    conn->StartClose();
    return;
  }
  if (conn->awaiting_response_) {
    // A request is already dispatched: an offloaded route computation is running
    // on the threadpool. The read phase is over, so the idle timer no longer
    // applies -- it bounds slow/idle READS and keep-alive idle, NOT compute. The
    // route computation is bounded (k is capped, the graph is fixed), so the
    // worker will finish and its completion callback writes the response on the
    // loop thread; the in-flight work holds a strong self-reference, so the
    // connection cannot leak meanwhile. Closing here would race that write and
    // silently drop a valid-but-slow response.
    //
    // A queued response write is different: once it is in flight, a client that
    // stops reading (a closed zero window) never completes uv_write, and the
    // connection would stay open forever, holding a slot against
    // max_connections. Bound that stall with the same idle timeout and close.
    if (conn->write_started_at_ms_ != 0 &&
        uv_now(conn->handle_.loop) - conn->write_started_at_ms_ >= conn->limits_.io_timeout_ms) {
      conn->StartClose();
      return;
    }
    // Not yet a write-stall (the worker is still computing): re-arm so the
    // write deadline is re-checked, rather than going silent for good. This does
    // not bound compute -- every re-arm re-tests the same condition.
    conn->RestartTimer();
    return;
  }
  if (!conn->closing_) {
    // Slow or idle client with nothing dispatched (slowloris, request deadline,
    // or an idle keep-alive): answer 408 per RFC 9110 §15.5.7 (SHOULD) so the
    // client learns why the connection dropped; the false keep_alive makes the
    // write close after.
    conn->awaiting_response_ = true;
    conn->WriteResponse(kStatusRequestTimeout, JsonError("request timeout"), false);
    return;
  }
  conn->StartClose();
}

bool Connection::RequestDeadlineExceeded() const {
  if (limits_.request_timeout_ms == 0 || request_started_at_ms_ == 0) {
    return false;
  }
  return uv_now(handle_.loop) - request_started_at_ms_ >= limits_.request_timeout_ms;
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
  // Stamp the hard request deadline clock. Idle keep-alive with no message in
  // flight leaves request_started_at_ms_ at 0 (ResetForNextRequest clears it).
  conn->request_started_at_ms_ = uv_now(conn->handle_.loop);
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
  // We never emit an interim 100 Continue; refuse Expect: 100-continue with 417
  // so the client does not stall waiting for a 100 that will never come.
  if (lower == "expect" && AsciiIeq(cur_value_, "100-continue")) {
    reject_status_ = kStatusExpectationFailed;
    reject_message_ = "Expect: 100-continue is not supported";
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
  // Early 413 when Content-Length already declares a body over the cap: refuse
  // before reading (or slow-dripping) the declared payload.
  if ((p->flags & F_CONTENT_LENGTH) != 0 && p->content_length > conn->limits_.max_body_bytes) {
    conn->reject_status_ = kStatusPayloadTooLarge;
    conn->reject_message_ = "request body exceeds the configured limit";
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
  head_request_ = AsciiIeq(req.method, "HEAD");
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
  // Peer address, for the Origin check and for logging.
  sockaddr_storage peer{};
  int peer_len = static_cast<int>(sizeof(peer));
  if (uv_tcp_getpeername(&handle_, reinterpret_cast<sockaddr*>(&peer), &peer_len) == 0) {
    char ip[INET6_ADDRSTRLEN] = {};
    if (peer.ss_family == AF_INET) {
      uv_ip4_name(reinterpret_cast<const sockaddr_in*>(&peer), ip, sizeof(ip));
    } else if (peer.ss_family == AF_INET6) {
      uv_ip6_name(reinterpret_cast<const sockaddr_in6*>(&peer), ip, sizeof(ip));
    }
    req.remote_address = ip;
  }
  handler_.Handle(shared_from_this(), req);
}

void Connection::WriteRaw(std::vector<uint8_t> payload, WriteMode mode) {
  if (closing_) {
    return;
  }
  if (write_started_at_ms_ == 0) {
    write_started_at_ms_ = uv_now(handle_.loop);
  }
  // uv_buf_init's length is unsigned (32-bit on common ABIs). A payload that
  // does not fit would truncate; refuse rather than write a partial frame.
  if (payload.size() > static_cast<size_t>((std::numeric_limits<unsigned>::max)())) {
    StartClose();
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
  // Pipelined bytes while awaiting (or same-buffer trailing request) force close
  // even when the handler / QueueWork asked for keep-alive.
  if (force_close_after_response_) {
    keep_alive = false;
  }
  // A HEAD response is the GET response minus its body: the framing headers
  // (Content-Length included) must describe the representation, so the body is
  // dropped at the frame level rather than by each handler.
  WriteRaw(BuildResponse(status, head_request_ ? "" : body, keep_alive, content_type, extra_headers,
                         elapsed_ms, /*content_length=*/body.size()),
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
  // for subsequent events rather than resetting/closing. Re-arm the idle timer:
  // open SSE streams are bounded by io_timeout_ms of inactivity (OnTimeout
  // closes when streaming_ is set).
  streaming_ = true;
  RestartTimer();
  WriteRaw(BuildStreamHeader(status, content_type, extra_headers), WriteMode::kStream);
}

void Connection::WriteEvent(const std::string& chunk) {
  if (closing_ || !streaming_) {
    return;
  }
  // Activity on the stream resets the idle clock.
  RestartTimer();
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
      conn->write_started_at_ms_ = 0;  // this chunk drained; nothing outstanding
      return;                          // an open stream: stay open for more events
    case WriteMode::kKeepAlive:
      conn->write_started_at_ms_ = 0;
      conn->ResetForNextRequest();
      return;
    case WriteMode::kClose:
      // Shut the write side down first and close only once the peer has been
      // told (uv_shutdown completes its callback after the FIN is queued). An
      // immediate uv_close while the client is still uploading sends a RST,
      // which discards the response already in the peer's receive buffer --
      // the client then sees ECONNRESET instead of the 413/431/417 it was sent.
      conn->write_started_at_ms_ = 0;
      conn->ShutdownThenClose();
      return;
  }
}

void Connection::OnHandleClosed(uv_handle_t* h) {
  auto* conn = static_cast<Connection*>(h->data);
  if (--conn->open_handles_ == 0) {
    // Notify the Server (live-connection accounting) before dropping self_: the
    // callback must not assume `conn` outlives this stack frame once reset runs.
    if (conn->on_closed_) {
      conn->on_closed_();
      conn->on_closed_ = nullptr;
    }
    conn->self_.reset();  // may delete conn; nothing may follow
  }
}

}  // namespace bf::http_server
