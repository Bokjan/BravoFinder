// jsonrpc.h — JSON-RPC 2.0 protocol constants, in one place.
//
// The pre-defined error codes below are not invented here: they are the reserved
// codes from JSON-RPC 2.0 §5.1 (https://www.jsonrpc.org/specification). Keeping
// them as named constants -- instead of -32600-style literals scattered across
// the dispatcher and the HTTP transport -- makes the intent readable and stops
// the codes from drifting. The -32000..-32099 "server error" range is reserved
// for implementation-defined errors and is not used here.

#pragma once

namespace bf::mcp::jsonrpc {

// The JSON-RPC version string every request/response envelope carries.
inline constexpr char kVersion[] = "2.0";

// §5.1 pre-defined error codes.
inline constexpr int kParseError     = -32700;  // Invalid JSON was received.
inline constexpr int kInvalidRequest = -32600;  // The JSON is not a valid Request object.
inline constexpr int kMethodNotFound = -32601;  // The method does not exist / is not available.
inline constexpr int kInvalidParams  = -32602;  // Invalid method parameter(s).
inline constexpr int kInternalError  = -32603;  // Internal JSON-RPC error.

}  // namespace bf::mcp::jsonrpc
