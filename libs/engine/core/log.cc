// SPDX-License-Identifier: LGPL-3.0-or-later

#include "core/log.h"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <utility>

namespace bf {

namespace {
// Process-wide default logger. nullptr = fully silent (the common case). Held
// as atomic<shared_ptr>: store on SetDefaultLogger, lock-free load on every
// LogImpl call. A logger replaced while a thread holds a copy stays alive via
// refcount until that copy is destroyed -- no use-after-free.
std::atomic<std::shared_ptr<Logger>> g_default_logger{nullptr};
}  // namespace

void SetDefaultLogger(std::shared_ptr<Logger> logger) {
  g_default_logger.store(std::move(logger), std::memory_order_release);
}

std::shared_ptr<Logger> DefaultLogger() { return g_default_logger.load(std::memory_order_acquire); }

std::string FormatLine(LogLevel level, std::string_view file, std::uint_least32_t line,
                       std::string_view message) {
  return std::format("[{}] {}:{} {}", kLevelPrefix[static_cast<int>(level)], file, line, message);
}

// ---- Logger ---------------------------------------------------------------

Logger::~Logger() = default;
void Logger::Flush() {}

// ---- StreamLogger ---------------------------------------------------------

void StreamLogger::Emit(LogLevel, std::string_view line) { out_ << line << '\n'; }
void StreamLogger::Flush() { out_.flush(); }

// StderrLogger constructor defined here (not inline) so <iostream>'s std::cerr
// is pulled into this one TU, not every includer of log.h.
StderrLogger::StderrLogger() : StreamLogger(std::cerr) {}

// ---- SyncLogger -----------------------------------------------------------

void SyncLogger::Emit(LogLevel level, std::string_view line) {
  std::lock_guard<std::mutex> lock(mu_);
  inner_.Emit(level, line);
}
void SyncLogger::Flush() {
  std::lock_guard<std::mutex> lock(mu_);
  inner_.Flush();
}

// ---- internal ------------------------------------------------------------

namespace internal {

namespace {
// Abort hook for BF_LOG_FATAL. nullptr => std::abort (production). Tests inject
// a throwing stand-in to verify the Emit+Flush+abort sequence in-process.
// Single-threaded install (test setup); reads happen on the fatal path only.
std::atomic<AbortFn> g_abort_hook{nullptr};
}  // namespace

void SetAbortHandler(AbortFn fn) { g_abort_hook.store(fn, std::memory_order_release); }

AbortFn GetAbortHandler() { return g_abort_hook.load(std::memory_order_acquire); }

[[noreturn]] void InvokeFatalAbort() {
  if (AbortFn hook = g_abort_hook.load(std::memory_order_acquire)) {
    hook();  // test stand-in throws here; the exception propagates out (a
             // [[noreturn]] function may exit via exception).
  }
  std::abort();  // default path + backstop if a hook ever returns.
}

}  // namespace internal

}  // namespace bf
