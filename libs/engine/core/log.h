// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Lightweight, opt-in logging for libs/engine.
//
// Design notes:
//   * Default-silent: the global default logger is nullptr, so the engine stays
//     quiet unless a caller installs a sink. The silent path is a mutex-guarded
//     shared_ptr copy (DefaultLogger) plus a relaxed atomic level check, and
//     never calls std::format.
//   * std::format + std::format_string<Args...> gives compile-time format/arg
//     checking (a mismatch fails the build, not a runtime crash).
//   * Source location is captured at the call site via the BF_LOG_* macros. A
//     macro-free primary API is impossible under C++20: a trailing defaulted
//     source_location parameter makes a trailing parameter pack deduce empty,
//     so location capture must go through a macro.
//   * A compile-time BF_LOG_MIN_LEVEL gate strips low-severity macros to
//     ((void)0); BF_LOG_FATAL is never stripped (callers may rely on its
//     "log then abort, never returns" control flow).
//   * The global default logger is a std::shared_ptr<Logger> guarded by an
//     internal mutex: SetDefaultLogger stores under the lock, DefaultLogger
//     copies under the lock on every LogImpl call, and a logger being replaced
//     stays alive (refcount) until in-flight users release it -- no
//     use-after-free. (Deliberately not std::atomic<std::shared_ptr<Logger>>:
//     that C++20 partial specialization is absent from Apple's pre-LLVM-19
//     libc++ and broke the Mac CI build; it is not lock-free on any mainstream
//     implementation anyway, so the mutex loses nothing real and is portable.)
//     This is the mainstream shape (spdlog/glog/log4cxx/Boost.Log all use a
//     global default logger). It is a deliberate, synchronized exception to
//     CLAUDE.md's "no static/global mutable state" rule: that rule targets v2's
//     bug pattern of cross-thread shared *unsynchronized* mutable state. Here
//     the mutex-guarded swap plus the per-sink mutex (SyncLogger) provide the
//     synchronization, so the v2 race does not arise. A thread-local logger was
//     rejected because threadpool workers default to nullptr and silently drop
//     logs; a global logger set once is visible to all workers, which is the
//     core benefit over thread-local.

#include <atomic>
#include <cstdint>
#include <format>
#include <memory>
#include <mutex>
#include <ostream>
#include <source_location>
#include <string>
#include <string_view>

namespace bf {

enum class LogLevel : int {
  kTrace = 1,
  kDebug,
  kInfo,
  kWarn,
  kError,
  kFatal,
};

// Abstract sink: implement to receive an already-formatted full line.
class Logger {
 public:
  virtual ~Logger();
  // `line` already carries "[LEVEL] file:line message"; the sink just writes it.
  virtual void Emit(LogLevel level, std::string_view line) = 0;
  // Flush the underlying output buffer. Default no-op; C stdio sinks may
  // override to fflush, StreamLogger overrides to out_.flush(). Called by
  // BF_LOG_FATAL after Emit and before abort so ostream buffers are not lost to
  // std::abort (which does not flush C++ streams).
  virtual void Flush();
  // min_level_ is atomic so Enabled (read on every LogImpl, including from
  // threadpool workers) races no UB against a concurrent set_min_level on the
  // same Logger. Relaxed ordering suffices: a stale level just over/under-logs
  // for one call, and the level is not used to synchronize any other state.
  bool Enabled(LogLevel l) const { return l >= min_level_.load(std::memory_order_relaxed); }
  void set_min_level(LogLevel l) { min_level_.store(l, std::memory_order_relaxed); }
  LogLevel min_level() const { return min_level_.load(std::memory_order_relaxed); }

 protected:
  std::atomic<LogLevel> min_level_{LogLevel::kDebug};
};

// Generic sink writing to any std::ostream (cout/cerr/file/ostringstream).
// Not thread-safe; wrap in SyncLogger for concurrent shared use. Holds a
// std::ostream& that must outlive this logger. Not final: StderrLogger derives
// from it to reuse Emit/Flush while pointing at std::cerr.
class StreamLogger : public Logger {
 public:
  explicit StreamLogger(std::ostream& out) : out_(out) {}
  void Emit(LogLevel, std::string_view line) override;
  void Flush() override;

 private:
  std::ostream& out_;
};

// Convenience subclass pointing at std::cerr. Its constructor is defined in the
// .cc so <iostream>'s std::cerr is pulled into one TU, not every includer.
class StderrLogger final : public StreamLogger {
 public:
  StderrLogger();
};

// Decorator: guards an inner Logger's Emit/Flush with a mutex so it can be
// called concurrently. Level filtering (Enabled/set_min_level/min_level) is
// inherited unchanged from the Logger base -- the decorator holds its own
// min_level_ (NOT delegated to inner_), and LogImpl filters via the global
// logger (this decorator) before reaching Emit; inner_ only does the write.
// Emit/Flush stay virtual (sink polymorphism); the level accessors are not, so
// the common filter check is a cheap non-virtual field compare, not a vcall.
class SyncLogger final : public Logger {
 public:
  explicit SyncLogger(Logger& inner) : inner_(inner) {}
  void Emit(LogLevel level, std::string_view line) override;
  void Flush() override;

 private:
  Logger& inner_;
  std::mutex mu_;
};

// Process-wide global default logger. nullptr by default = fully silent.
// Held as a mutex-guarded shared_ptr (see log.cc): set/restore under the lock,
// readers take a refcounted copy, and a replaced logger stays alive until
// in-flight users release it -- no use-after-free.
void SetDefaultLogger(std::shared_ptr<Logger> logger);
std::shared_ptr<Logger> DefaultLogger();

// RAII: install a global logger, restore the previous one on scope exit
// (exception-safe). For single-threaded CLI / test isolation. Concurrent set is
// not safe -- assume install happens on a single thread (main start or test
// setup).
class ScopedLogger {
 public:
  explicit ScopedLogger(std::shared_ptr<Logger> logger) : prev_(DefaultLogger()) {
    SetDefaultLogger(std::move(logger));
  }
  ~ScopedLogger() { SetDefaultLogger(prev_); }
  ScopedLogger(const ScopedLogger&) = delete;
  ScopedLogger& operator=(const ScopedLogger&) = delete;

 private:
  std::shared_ptr<Logger> prev_;
};

// Formats one log line as "[LEVEL] file:line message". Exposed so sinks/tests
// can reuse the exact shape.
std::string FormatLine(LogLevel level, std::string_view file, std::uint_least32_t line,
                       std::string_view message);

// Short prefixes for each LogLevel (indexed by int(LogLevel)); index 0 unused.
inline constexpr const char* kLevelPrefix[] = {"",     "TRACE", "DEBUG", "INFO",
                                               "WARN", "ERROR", "FATAL"};

// Macro-free internal entry: source_location is an explicit leading parameter
// (no default); the parameter pack Args&&... is trailing, so it deduces
// normally (a trailing defaulted source_location would make the pack deduce
// empty -- the death of the earlier "macro-free API" draft). std::format_string
// does the compile-time format/arg-type check here. std::format runs only after
// Enabled passes -- the silent path is a mutex-guarded shared_ptr copy
// (DefaultLogger) plus a relaxed atomic level load.
template <typename... Args>
void LogImpl(std::source_location loc, LogLevel level, std::format_string<Args...> fmt,
             Args&&... args) {
  std::shared_ptr<Logger> logger = DefaultLogger();
  if (!logger || !logger->Enabled(level)) {
    return;
  }
  logger->Emit(level, FormatLine(level, loc.file_name(), loc.line(),
                                 std::format(fmt, std::forward<Args>(args)...)));
}

namespace internal {

using AbortFn = void (*)();

// Abort hook for BF_LOG_FATAL. nullptr => std::abort (production behavior
// unchanged). Tests inject a throwing stand-in to verify the Emit+Flush+abort
// sequence in-process. Single-threaded install (test setup); reads happen on
// the fatal path only.
void SetAbortHandler(AbortFn fn);
AbortFn GetAbortHandler();

// [[noreturn]]: calls the installed hook (default std::abort). Marked noreturn
// so the compiler treats code after BF_LOG_FATAL as unreachable. A test
// stand-in that throws is allowed: a [[noreturn]] function may exit via
// exception (throw is a non-local exit, not a normal return). The trailing
// std::abort is a backstop in case a hook ever returns.
[[noreturn]] void InvokeFatalAbort();

}  // namespace internal

// ---------------------------------------------------------------------------
// Call-site macros. std::source_location::current() written here (the call
// site) captures the caller's location, then forwards to LogImpl. This is the
// only viable shape: location capture must go through a macro (a macro-free
// primary API is impossible under C++20 -- see LogImpl). The same macros carry
// the compile-time strip gate (BF_LOG_MIN_LEVEL): levels below the threshold
// expand to ((void)0), their format string never entering the binary.
// __VA_OPT__(,) handles zero extra arguments (e.g. BF_LOG_DEBUG("plain msg")).
// MSVC needs /Zc:preprocessor for __VA_OPT__ (VS2019 16.5+; CI matrix has it).
// The top-level CMakeLists.txt adds /Zc:preprocessor globally, so any TU
// including this header (bravofinder sources, apps, tests) gets it -- do not
// scope it per-target or consumers like bf_tests will be missed.
// ---------------------------------------------------------------------------
#ifndef BF_LOG_MIN_LEVEL
#define BF_LOG_MIN_LEVEL 1  // default kTrace: keep everything
#endif

#if BF_LOG_MIN_LEVEL <= 1
#define BF_LOG_TRACE(...) \
  ::bf::LogImpl(std::source_location::current(), ::bf::LogLevel::kTrace __VA_OPT__(, ) __VA_ARGS__)
#else
#define BF_LOG_TRACE(...) ((void)0)
#endif

#if BF_LOG_MIN_LEVEL <= 2
#define BF_LOG_DEBUG(...) \
  ::bf::LogImpl(std::source_location::current(), ::bf::LogLevel::kDebug __VA_OPT__(, ) __VA_ARGS__)
#else
#define BF_LOG_DEBUG(...) ((void)0)
#endif

#if BF_LOG_MIN_LEVEL <= 3
#define BF_LOG_INFO(...) \
  ::bf::LogImpl(std::source_location::current(), ::bf::LogLevel::kInfo __VA_OPT__(, ) __VA_ARGS__)
#else
#define BF_LOG_INFO(...) ((void)0)
#endif

#if BF_LOG_MIN_LEVEL <= 4
#define BF_LOG_WARN(...) \
  ::bf::LogImpl(std::source_location::current(), ::bf::LogLevel::kWarn __VA_OPT__(, ) __VA_ARGS__)
#else
#define BF_LOG_WARN(...) ((void)0)
#endif

#if BF_LOG_MIN_LEVEL <= 5
#define BF_LOG_ERROR(...) \
  ::bf::LogImpl(std::source_location::current(), ::bf::LogLevel::kError __VA_OPT__(, ) __VA_ARGS__)
#else
#define BF_LOG_ERROR(...) ((void)0)
#endif

// BF_LOG_FATAL (=6) is never stripped (always compiled in): callers may rely on
// its "log then abort, never returns" control flow; stripping would break that
// assumption (loadmaster's LOG_FATAL does the same). Flush before abort -- the
// ostream buffer would otherwise be dropped by std::abort (it does not flush C++
// streams). Abort goes through internal::InvokeFatalAbort (an injectable hook:
// default std::abort, tests inject a throw stand-in).
#define BF_LOG_FATAL(...)                                             \
  do {                                                                \
    ::bf::LogImpl(std::source_location::current(),                    \
                  ::bf::LogLevel::kFatal __VA_OPT__(, ) __VA_ARGS__); \
    if (auto _bf_l = ::bf::DefaultLogger()) {                         \
      _bf_l->Flush();                                                 \
    }                                                                 \
    ::bf::internal::InvokeFatalAbort();                               \
  } while (false)

}  // namespace bf
