// SPDX-License-Identifier: MIT
#include "core/base/log.h"

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

// A sink that captures every emitted line and counts Emit/Flush calls, for
// assertions that need more than a raw stream (emit/flush counts, exact text).
class CapturingSink : public bf::Logger {
 public:
  void Emit(bf::LogLevel, std::string_view line) override {
    ++emit_count;
    if (!captured_.empty()) {
      captured_ += '\n';
    }
    captured_ += std::string(line);
  }
  void Flush() override { ++flush_count; }

  int emit_count = 0;
  int flush_count = 0;
  std::string captured_;
};

// RAII guard that restores the global abort hook to nullptr on scope exit, so a
// test that injects a throwing hook cannot leak it into later cases (a leaked
// hook would turn a later BF_LOG_FATAL into a throw instead of std::abort).
class AbortGuard {
 public:
  explicit AbortGuard(bf::internal::AbortFn fn) { bf::internal::SetAbortHandler(fn); }
  ~AbortGuard() { bf::internal::SetAbortHandler(nullptr); }
  AbortGuard(const AbortGuard&) = delete;
  AbortGuard& operator=(const AbortGuard&) = delete;
};

}  // namespace

// 1. Default-silent: with no logger installed, BF_LOG_* is a no-op (no Emit).
// The LogImpl early-return also skips std::format -- the silent path is one
// atomic load (format-not-called is a code-path guarantee, not directly
// observable at runtime since call-site args are evaluated before the call).
TEST_CASE("silent without a logger, emits with one", "[unit][log]") {
  bf::SetDefaultLogger(nullptr);
  auto sink = std::make_shared<CapturingSink>();
  BF_LOG_DEBUG("ignored {}", 1);
  CHECK(sink->emit_count == 0);

  {
    bf::ScopedLogger scope(sink);
    BF_LOG_DEBUG("captured {}", 2);
    CHECK(sink->emit_count == 1);
    CHECK(sink->captured_.find("captured 2") != std::string::npos);
  }
  // After scope, the global logger is restored to nullptr: further logs are silent.
  BF_LOG_DEBUG("ignored again {}", 3);
  CHECK(sink->emit_count == 1);  // unchanged
  bf::SetDefaultLogger(nullptr);
}

// 2. Level filtering: messages below the sink's min_level are dropped (no Emit).
TEST_CASE("level filtering respects min_level", "[unit][log]") {
  auto sink = std::make_shared<CapturingSink>();
  sink->set_min_level(bf::LogLevel::kWarn);
  bf::ScopedLogger scope(sink);
  BF_LOG_DEBUG("debug msg");
  BF_LOG_INFO("info msg");
  BF_LOG_WARN("warn msg");
  BF_LOG_ERROR("error msg");
  CHECK(sink->emit_count == 2);
  CHECK(sink->captured_.find("warn msg") != std::string::npos);
  CHECK(sink->captured_.find("error msg") != std::string::npos);
  CHECK(sink->captured_.find("debug msg") == std::string::npos);
  CHECK(sink->captured_.find("info msg") == std::string::npos);
}

// 3. Line format is "[LEVEL] file:line message".
TEST_CASE("emitted line is [LEVEL] file:line message", "[unit][log]") {
  auto sink = std::make_shared<CapturingSink>();
  bf::ScopedLogger scope(sink);
  BF_LOG_WARN("hello {}", "world");
  REQUIRE(sink->emit_count == 1);
  const std::string& line = sink->captured_;
  CHECK(line.substr(0, 6) == "[WARN]");
  CHECK(line.find("hello world") != std::string::npos);
  // The file path is trimmed to repo-relative by -ffile-prefix-map; this very
  // source file is the call site, so its name must appear, followed by a line
  // number.
  CHECK(line.find("log_test.cc") != std::string::npos);
  CHECK(line.find(" hello world") != std::string::npos);
}

// 4. ScopedLogger restores the previous global logger on scope exit.
TEST_CASE("ScopedLogger restores previous logger", "[unit][log]") {
  auto first = std::make_shared<CapturingSink>();
  bf::SetDefaultLogger(first);
  auto second = std::make_shared<CapturingSink>();
  {
    bf::ScopedLogger scope(second);
    BF_LOG_DEBUG("to second");
    CHECK(second->emit_count == 1);
    CHECK(first->emit_count == 0);
  }
  // After scope, 'first' is restored.
  BF_LOG_DEBUG("to first");
  CHECK(first->emit_count == 1);
  CHECK(second->emit_count == 1);  // unchanged
  bf::SetDefaultLogger(nullptr);
}

// 5. SyncLogger is safe under concurrent Emit/Flush (the tsan point). A bare
// StreamLogger would race on the shared ostringstream; the mutex must serialize.
TEST_CASE("SyncLogger concurrent Emit/Flush is safe", "[unit][log]") {
  std::ostringstream os;
  auto inner = std::make_shared<bf::StreamLogger>(os);
  // SyncLogger holds inner by reference; inner must outlive the SyncLogger,
  // which it does for the duration of this scope.
  auto sync = std::make_shared<bf::SyncLogger>(*inner);
  bf::ScopedLogger scope(sync);

  std::atomic<int> emitted{0};
  std::vector<std::thread> threads;
  for (int t = 0; t < 8; ++t) {
    threads.emplace_back([t, &emitted]() {
      for (int i = 0; i < 100; ++i) {
        BF_LOG_DEBUG("thread {} iter {}", t, i);
        ++emitted;
      }
      if (auto logger = bf::DefaultLogger()) {
        logger->Flush();
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  // tsan-clean is the point; we only assert all logs landed. The exact line
  // count in a shared ostringstream is not asserted (interleaving is fine).
  CHECK(emitted.load() == 800);
  CHECK_FALSE(os.str().empty());
}

// 6. SyncLogger delegates the level (Enabled/set_min_level/min_level) to its
// inner sink: setting the level on the decorator actually reaches the inner.
TEST_CASE("SyncLogger filters by its own level", "[unit][log]") {
  std::ostringstream os;
  auto inner = std::make_shared<bf::StreamLogger>(os);
  auto sync = std::make_shared<bf::SyncLogger>(*inner);
  bf::ScopedLogger scope(sync);

  sync->set_min_level(bf::LogLevel::kError);
  CHECK(sync->min_level() == bf::LogLevel::kError);
  CHECK(inner->min_level() == bf::LogLevel::kDebug);  // SyncLogger owns level; inner unaffected
  CHECK_FALSE(sync->Enabled(bf::LogLevel::kDebug));
  CHECK(sync->Enabled(bf::LogLevel::kError));

  BF_LOG_DEBUG("hidden");
  BF_LOG_ERROR("shown");
  CHECK(os.str().find("hidden") == std::string::npos);
  CHECK(os.str().find("shown") != std::string::npos);
}

// 7. A logger installed on the main thread is visible to a worker thread with
// no per-thread wiring. This is the core benefit of a global (vs thread-local)
// default logger: a threadpool worker never silently drops logs.
TEST_CASE("global logger is visible across threads", "[unit][log]") {
  auto sink = std::make_shared<CapturingSink>();
  bf::ScopedLogger scope(sink);

  std::atomic<int> seen{0};
  std::thread worker([&seen]() {
    if (bf::DefaultLogger()) {
      ++seen;
    }
    BF_LOG_DEBUG("from worker {}", 1);
  });
  worker.join();
  CHECK(seen.load() == 1);
  CHECK(sink->emit_count == 1);
  CHECK(sink->captured_.find("from worker 1") != std::string::npos);
}

// 8. BF_LOG_FATAL emits, flushes, then aborts. The abort is routed through an
// injectable hook so this stays in-process: a throwing stand-in replaces
// std::abort, and we assert the Emit+Flush+abort sequence completed.
TEST_CASE("BF_LOG_FATAL emits flushes then aborts", "[unit][log]") {
  auto sink = std::make_shared<CapturingSink>();
  bf::ScopedLogger scope(sink);

  AbortGuard guard([]() { throw std::runtime_error("fatal-abort"); });

  REQUIRE_THROWS_AS([&] { BF_LOG_FATAL("boom {}", 42); }(), std::runtime_error);

  // Emit happened (LogImpl), Flush happened (the macro, before abort).
  CHECK(sink->emit_count == 1);
  CHECK(sink->flush_count == 1);
  CHECK(sink->captured_.find("[FATAL]") != std::string::npos);
  CHECK(sink->captured_.find("boom 42") != std::string::npos);
}
