// SPDX-License-Identifier: MIT
#pragma once

#include "dispatcher.h"
#include "registry.h"

namespace bf::mcp {

// StdioRunner speaks MCP over stdio as JSON-RPC 2.0: one request per line from
// stdin, one response per line to stdout. It owns only the transport concerns --
// bounded line reading, JSON parsing, and writing the response -- and delegates
// every protocol decision to a Dispatcher. The heavy work runs inline on this
// thread (stdio is a single local client), so there is no offload here; the HTTP
// transport (mcp_http) offloads the same Dispatcher instead.
class StdioRunner {
 public:
  // Takes the registry to serve from. The registry must outlive the runner.
  // Each database it holds is read-only per NavDatabase thread-safety contract.
  explicit StdioRunner(bf::service::NavDatabaseRegistry& registry) : dispatcher_(registry) {}

  // Run the stdio request/response loop until stdin closes. Returns the process
  // exit status.
  int Run();

 private:
  Dispatcher dispatcher_;
};

}  // namespace bf::mcp
