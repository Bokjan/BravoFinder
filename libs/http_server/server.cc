// SPDX-License-Identifier: MIT
// server.cc — bind/listen and accept-to-Connection for the shared HTTP core.

#include "server.h"

#include <cstdint>

#include "conn.h"

namespace bf::http_server {

Server::Server(uv_loop_t* loop, RequestHandler& handler, const Limits& limits)
    : loop_(loop), handler_(handler), limits_(limits) {
  uv_tcp_init(loop_, &handle_);
  handle_.data = this;
}

Server::~Server() { Close(); }

int Server::Listen(const std::string& host, int port) {
  struct sockaddr_in addr;
  int rc = uv_ip4_addr(host.c_str(), port, &addr);
  if (rc != 0) {
    return rc;
  }
  rc = uv_tcp_bind(&handle_, reinterpret_cast<const struct sockaddr*>(&addr), 0);
  if (rc != 0) {
    return rc;
  }
  return uv_listen(reinterpret_cast<uv_stream_t*>(&handle_), /*backlog=*/128, OnNewConnection);
}

int Server::BoundPort() const {
  struct sockaddr_storage ss;
  int len = sizeof(ss);
  if (uv_tcp_getsockname(&handle_, reinterpret_cast<struct sockaddr*>(&ss), &len) != 0) {
    return -1;
  }
  if (ss.ss_family != AF_INET) {
    return -1;
  }
  // sin_port is in network byte order; read its two bytes directly so no
  // ntohs / platform networking header is needed.
  const auto* in4 = reinterpret_cast<const struct sockaddr_in*>(&ss);
  const auto* bytes = reinterpret_cast<const unsigned char*>(&in4->sin_port);
  return (static_cast<int>(bytes[0]) << 8) | static_cast<int>(bytes[1]);
}

void Server::Close() {
  // handle_ is uv_tcp_init'd in the constructor, so it is always a valid handle
  // here even if Listen failed; guard against a double close.
  auto* h = reinterpret_cast<uv_handle_t*>(&handle_);
  if (uv_is_closing(h) == 0) {
    uv_close(h, nullptr);
  }
}

void Server::OnNewConnection(uv_stream_t* server, int status) {
  if (status != 0) {
    return;  // accept failed at the libuv level; nothing to clean up yet
  }
  auto* self = static_cast<Server*>(server->data);
  std::shared_ptr<Connection> conn =
      Connection::Create(server->loop, self->handler_, self->limits_);
  if (uv_accept(server, conn->stream()) == 0) {
    conn->Start();
  } else {
    conn->Close();  // could not accept into the handle; close and free it
  }
}

}  // namespace bf::http_server
