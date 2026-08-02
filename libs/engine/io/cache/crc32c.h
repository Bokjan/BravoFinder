// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// CRC-32C (Castagnoli) for .bfdb integrity checks.
//
// Each .bfdb section body and each CIFP segment carries a CRC-32C of its own
// bytes (stored alongside its offset/length in the section table or CIFP
// directory), plus a CRC of the global string pool. This guards against silent
// media corruption -- a bit flip that lands on another *valid* value (e.g. an
// edge distance 10.0 -> 9.99), which the field-level decode fuses cannot catch
// because the value stays in range, the enum stays valid, and the pool
// reference still resolves. CRC32C is the fuse for "the bytes are intact";
// the decode fuses remain the fuse for "the bytes are self-consistent".
//
// CRC-32C uses the Castagnoli polynomial 0x1EDC6F41, reflected as 0x82F63B78.
// It is the same polynomial as x86 SSE4.2 `crc32` and ARMv8 CRC32 instructions,
// so a hardware fast path (_mm_crc32_u32 / __crc32cd) could drop in later with
// identical seed semantics. The table implementation below is the portable
// default -- both x86-64 and arm64 run in CI and release builds, and CRC is a
// one-shot O(filesize) pass over a ~0.20s total load, so the table is not the
// bottleneck; a hardware path is an optional future optimization, not needed
// for correctness or throughput today.
//
// The table is a constexpr std::array (immutable, no global mutable state),
// satisfying the engine's no-static-mutable rule.

#include <array>
#include <cstdint>
#include <span>

namespace bf {

class Crc32C {
 public:
  // Full checksum of `data`. Equivalent to Update(0, data).
  static uint32_t Compute(std::span<const uint8_t> data) { return Update(0, data); }

  // Continue a checksum: fold `data` into the running `crc`. Use to checksum a
  // logical span assembled from non-contiguous reads (e.g. the CIFP directory
  // prefix, read as a 4-byte count followed by the directory rows) without
  // copying the parts into one buffer first.
  static uint32_t Update(uint32_t crc, std::span<const uint8_t> data) {
    uint32_t c = crc ^ 0xFFFFFFFFu;
    for (uint8_t b : data) {
      c = kTable[(c ^ b) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
  }

 private:
  // CRC-32C Castagnoli table (reflected polynomial 0x82F63B78), generated once
  // at compile time. An in-class IIFE -- not a member function -- initializes it,
  // so the initializer does not reference the class before it is complete (an
  // in-class `kTable = MakeTable()` is not yet a constant expression under GCC's
  // strict reading, and a constexpr static member must be initialized in-class).
  static constexpr std::array<uint32_t, 256> kTable = [] {
    constexpr uint32_t kPoly = 0x82F63B78u;
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        c = (c & 1u) ? (kPoly ^ (c >> 1)) : (c >> 1);
      }
      t[i] = c;
    }
    return t;
  }();
};

}  // namespace bf
