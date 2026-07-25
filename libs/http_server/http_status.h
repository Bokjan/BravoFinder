// SPDX-License-Identifier: MIT
// http_status.h — canonical HTTP status codes for the transport core and the
// apps that speak HTTP (bf-http REST, bf-mcp over HTTP). Centralized so no
// endpoint hard-codes a bare integer status. The bf::service layer keeps its own
// copy of the codes it needs (queries.cc kOk/kNotFound/kUnprocessable,
// handlers.cc kBadRequest) because it must not depend on the transport core;
// the two sets are deliberately independent, mirroring the dual JsonError.

#pragma once

namespace bf::http_server {

// 0 is not a real HTTP status; it is the "no rejection yet" sentinel for a
// connection's per-request reject_status_ latch.
inline constexpr int kStatusNone = 0;

inline constexpr int kStatusOk = 200;
inline constexpr int kStatusAccepted = 202;
inline constexpr int kStatusBadRequest = 400;
inline constexpr int kStatusNotFound = 404;
inline constexpr int kStatusMethodNotAllowed = 405;
inline constexpr int kStatusRequestTimeout = 408;
inline constexpr int kStatusPayloadTooLarge = 413;
inline constexpr int kStatusUriTooLong = 414;
inline constexpr int kStatusUnprocessableEntity = 422;
inline constexpr int kStatusRequestHeaderFieldsTooLarge = 431;
inline constexpr int kStatusInternalServerError = 500;
inline constexpr int kStatusServiceUnavailable = 503;

// Status-class boundaries, for family checks (2xx success, 3xx redirection,
// 4xx client error, 5xx server error) rather than a specific code.
inline constexpr int kStatusSuccessMin = 200;
inline constexpr int kStatusRedirectionMin = 300;
inline constexpr int kStatusClientErrorMin = 400;
inline constexpr int kStatusServerErrorMin = 500;

}  // namespace bf::http_server
