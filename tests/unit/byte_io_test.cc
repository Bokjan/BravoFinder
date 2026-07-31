// SPDX-License-Identifier: MIT
#include "io/cache/byte_io.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace {

// L3 hardening: ByteWriter::Str guards its U32 length prefix. A string longer
// than 4 GiB cannot be represented and, if serialized, would desynchronize the
// decoder; the writer now sets ok() false instead of truncating the length. The
// >4 GiB branch itself is unreachable for real AIRAC data and cannot be exercised
// in a fast unit test (it would require a 4 GiB allocation), so these cases lock
// the observable contract around it: ok() stays true on the happy path, Str
// round-trips a length-prefixed string, and the reader rejects a length prefix
// longer than the bytes present -- the symmetric protection on the decode side.

TEST_CASE("ByteWriter/ByteReader: normal writes keep ok() true and Str round-trips",
          "[unit][byte_io]") {
  std::vector<uint8_t> buf;
  bf::ByteWriter w(buf);
  CHECK(w.ok());  // starts true

  w.U32(0xDEADBEEF);
  w.Str("hello");
  w.I16(-1234);
  CHECK(w.ok());  // happy path never trips the L3 guard

  bf::ByteReader r(buf);
  CHECK(r.U32() == 0xDEADBEEF);
  const std::string s = r.Str();
  CHECK(r.ok());
  CHECK(s == "hello");
  CHECK(r.I16() == -1234);
  CHECK(r.ok());
  CHECK(r.remaining() == 0);  // exactly consumed, no desync
}

TEST_CASE("ByteReader::Str: a length prefix longer than remaining bytes fails cleanly",
          "[unit][byte_io]") {
  // Hand-craft a length-prefixed string claiming 10 bytes but supply only 3; the
  // reader must refuse rather than read past the buffer (the mirror of the L3
  // writer guard -- a length and its payload must agree on both sides).
  std::vector<uint8_t> buf;
  bf::ByteWriter w(buf);
  w.U32(10);
  w.Bytes(reinterpret_cast<const uint8_t*>("abc"), 3);
  bf::ByteReader r(buf);
  const std::string s = r.Str();
  CHECK_FALSE(r.ok());
  CHECK(s.empty());
}

}  // namespace
