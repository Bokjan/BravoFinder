// SPDX-License-Identifier: MIT
#include <catch2/catch_test_macros.hpp>
#include <limits>

#include "core/graph/astar.h"

// White-box tests for SearchWorkspace: the O(1) generation-stamp reset that lets
// Yen reuse one scratch across hundreds of spur searches. The public search
// entry points only exercise it as a black box, so the stamp invariants (a fresh
// generation reads initial values, a stale slot is cleared on first touch, and
// the closed stamp is independent of the value stamp) get no direct coverage
// there. These probe them directly. A slot must only be read after the first
// NextGeneration(): generation_ starts at 0 and the stamp arrays default to 0,
// so before any bump a slot's stamp spuriously matches the generation.

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

TEST_CASE("search workspace: a fresh generation reads initial values", "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(8);
  ws.NextGeneration();  // must bump before reading any slot
  for (int v = 0; v < 8; ++v) {
    CHECK(ws.G(v) == kInf);
    CHECK(ws.Prev(v) == -1);
    CHECK_FALSE(ws.Closed(v));
  }
}

TEST_CASE("search workspace: relax records cost geo and predecessor", "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(4);
  ws.NextGeneration();
  ws.Relax(2, /*g=*/5.0, /*geo=*/4.0, /*prev=*/1, /*inbound=*/90.0);
  CHECK(ws.G(2) == 5.0);
  CHECK(ws.Geo(2) == 4.0);
  CHECK(ws.Prev(2) == 1);
  CHECK(ws.Inbound(2) == 90.0);
  CHECK_FALSE(ws.Closed(2));  // relax records cost but does not close
  // A vertex never relaxed this generation still reads initial.
  CHECK(ws.G(3) == kInf);
  CHECK(ws.Prev(3) == -1);
  CHECK(ws.Inbound(3) == -1.0);
}

TEST_CASE("search workspace: closed is an independent generation stamp", "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(4);
  ws.NextGeneration();
  ws.MarkClosed(1);
  CHECK(ws.Closed(1));
  CHECK_FALSE(ws.Closed(0));
  // Closed carries its own stamp, so a vertex can be closed without a cost slot
  // ever being written (the dual-slot semantics).
  CHECK(ws.G(1) == kInf);
}

TEST_CASE("search workspace: next generation clears all state in constant time",
          "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(4);
  ws.NextGeneration();
  ws.Relax(0, 1.0, 1.0, -1, /*inbound=*/-1.0);
  ws.Relax(1, 2.0, 2.0, 0, /*inbound=*/45.0);
  ws.MarkClosed(0);
  // Bumping the generation logically resets every slot without touching memory.
  ws.NextGeneration();
  for (int v = 0; v < 4; ++v) {
    CHECK(ws.G(v) == kInf);
    CHECK(ws.Prev(v) == -1);
    CHECK_FALSE(ws.Closed(v));
  }
}

TEST_CASE("search workspace: a stale slot is cleared on first touch in a new generation",
          "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(4);
  ws.NextGeneration();
  ws.Relax(2, 5.0, 4.0, 1, /*inbound=*/90.0);  // dirties slot 2 in generation 1
  ws.NextGeneration();                         // generation 2: slot 2 is now stale
  // Re-relaxing must overwrite cleanly, not carry over the stale geo/prev.
  ws.Relax(2, 9.0, 8.0, -1, /*inbound=*/-1.0);
  CHECK(ws.G(2) == 9.0);
  CHECK(ws.Geo(2) == 8.0);
  CHECK(ws.Prev(2) == -1);
  CHECK(ws.Inbound(2) == -1.0);
}

TEST_CASE("search workspace: stamps stay correct across many generations", "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(2);
  for (int gen = 0; gen < 5; ++gen) {
    ws.NextGeneration();
    CHECK(ws.G(0) == kInf);  // clean at the start of every generation
    ws.Relax(0, gen + 1.0, 0.0, -1, /*inbound=*/-1.0);
    CHECK(ws.G(0) == gen + 1.0);
  }
}

TEST_CASE("search workspace: reset grows without dropping the active generation",
          "[unit][workspace]") {
  bf::SearchWorkspace ws;
  ws.Reset(2);
  ws.NextGeneration();
  ws.Relax(1, 3.0, 3.0, 0, /*inbound=*/-1.0);
  // Reset never shrinks and does not touch the value arrays or the generation
  // counter, so a grow preserves slots already written this generation.
  ws.Reset(16);
  CHECK(ws.G(1) == 3.0);
  CHECK(ws.Prev(1) == 0);
  // Newly added slots read as initial in the current generation.
  CHECK(ws.G(10) == kInf);
  CHECK(ws.Prev(10) == -1);
}

}  // namespace
