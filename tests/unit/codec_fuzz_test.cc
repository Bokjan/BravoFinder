#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "io/cache/graph_codec.h"
#include "io/cache/graph_snapshot.h"
#include "io/cache/nav_detail_codec.h"

// Negative-path guardrail for the cache section decoders. The contract is that a
// malformed / truncated / over-referenced body yields Result::Err (kCacheCorrupt)
// and NEVER crashes, over-allocates, or reads out of bounds -- a corrupt .bfdb
// must degrade to a clean error, not a crash or a wrong route. These run under
// ASan in the debug preset, so any out-of-bounds read during decode fails here.

namespace {

// An arbitrary small string-pool blob: NUL-separated entries. Not a valid pool
// for any particular body; the decoders must tolerate references that miss.
const std::string kPool("\0KJFK\0CANDR\0KLAX\0", 17);

TEST_CASE("GraphCodec::Decode: empty and tiny inputs error, never crash", "[codec_fuzz]") {
  const char empty[1] = {0};
  CHECK_FALSE(bf::GraphCodec::Decode(empty, 0, kPool.data(), kPool.size()));
  const char tiny[3] = {0x01, 0x02, 0x03};
  CHECK_FALSE(bf::GraphCodec::Decode(tiny, sizeof(tiny), kPool.data(), kPool.size()));
}

TEST_CASE("NavDetailCodec::Decode: empty and tiny inputs error, never crash", "[codec_fuzz]") {
  const char empty[1] = {0};
  CHECK_FALSE(bf::NavDetailCodec::Decode(empty, 0, kPool.data(), kPool.size()));
  const char tiny[3] = {0x7f, 0x00, 0x42};
  CHECK_FALSE(bf::NavDetailCodec::Decode(tiny, sizeof(tiny), kPool.data(), kPool.size()));
}

TEST_CASE("cache decoders: random and truncated bytes never crash", "[codec_fuzz]") {
  std::mt19937 rng(0xC0FFEEu);  // fixed seed: deterministic
  std::uniform_int_distribution<int> byte(0, 255);
  std::uniform_int_distribution<int> len(1, 512);
  for (int iter = 0; iter < 3000; ++iter) {
    std::vector<char> buf(static_cast<size_t>(len(rng)));
    for (char& c : buf) {
      c = static_cast<char>(byte(rng));
    }
    // Only termination + no OOB (ASan) is asserted. A result may be Ok (the bytes
    // happened to form a self-consistent snapshot) or Err; both are fine here.
    auto g = bf::GraphCodec::Decode(buf.data(), buf.size(), kPool.data(), kPool.size());
    (void)g;
    auto d = bf::NavDetailCodec::Decode(buf.data(), buf.size(), kPool.data(), kPool.size());
    (void)d;
  }
  SUCCEED();
}

}  // namespace
