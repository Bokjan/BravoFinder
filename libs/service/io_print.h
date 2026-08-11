// SPDX-License-Identifier: MIT
// io_print.h — shared stdout/stderr helpers for bf / bf-http / bf-mcp.
//
// Call sites must not stream to std::cout / std::cerr directly: use these so
// prefixes (error: / warning:) and newline policy stay consistent across the
// three front-ends. Engine diagnostics still go through BF_LOG_* / StderrLogger;
// that channel is separate from these user-facing prints.

#pragma once

#include <format>
#include <iostream>
#include <string_view>
#include <utility>

#include "render.h"  // kTextErrorPrefix

namespace bf::service {

inline constexpr std::string_view kTextWarningPrefix = "warning: ";

namespace detail {

inline void WriteStream(std::ostream& out, std::string_view text, bool ensure_newline) {
  out << text;
  if (ensure_newline && (text.empty() || text.back() != '\n')) {
    out << '\n';
  }
}

}  // namespace detail

// User/config failure: "error: <message>\n" on stderr (scripts grep the prefix).
inline void PrintError(std::string_view message) {
  std::cerr << kTextErrorPrefix << message << '\n';
}

template <class... Args>
void PrintError(std::format_string<Args...> fmt, Args&&... args) {
  PrintError(std::format(fmt, std::forward<Args>(args)...));
}

// Non-fatal notice: "warning: <message>\n" on stderr.
inline void PrintWarning(std::string_view message) {
  std::cerr << kTextWarningPrefix << message << '\n';
}

template <class... Args>
void PrintWarning(std::format_string<Args...> fmt, Args&&... args) {
  PrintWarning(std::format(fmt, std::forward<Args>(args)...));
}

// Process status (e.g. "listening on ...") on stderr, no prefix.
inline void PrintStatus(std::string_view message) {
  detail::WriteStream(std::cerr, message, /*ensure_newline=*/true);
}

template <class... Args>
void PrintStatus(std::format_string<Args...> fmt, Args&&... args) {
  PrintStatus(std::format(fmt, std::forward<Args>(args)...));
}

// Already-rendered success body on stdout (no extra prefix).
inline void PrintStdout(std::string_view text, bool ensure_newline = false) {
  detail::WriteStream(std::cout, text, ensure_newline);
}

// Already-rendered failure body on stderr (HandlerResult.body). Does NOT add
// kTextErrorPrefix — text bodies from RenderError already carry it; JSON bodies
// are {"error":...} and must not be wrapped again.
inline void PrintStderr(std::string_view text, bool ensure_newline = false) {
  detail::WriteStream(std::cerr, text, ensure_newline);
}

}  // namespace bf::service
