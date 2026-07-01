#pragma once

// Internal little-endian byte serialization helpers shared by the cache
// writers/readers (bfdb_cache, cifp_cache). Not a public API.
//
// Integers are emitted least-significant-byte first, so output is identical
// regardless of host endianness. Floats are written via their IEEE-754 bit
// pattern (memcpy to an unsigned integer of the same width), which every
// current platform shares, so values round-trip exactly across architectures.

#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

namespace bf {

// Little-endian writer appending to an in-memory byte buffer.
class ByteWriter {
 public:
  explicit ByteWriter(std::string& out) : out_(out) {}

  void U8(uint8_t v) { out_.push_back(static_cast<char>(v)); }

  void U16(uint16_t v) {
    U8(static_cast<uint8_t>(v));
    U8(static_cast<uint8_t>(v >> 8));
  }

  void U32(uint32_t v) {
    for (int i = 0; i < 4; ++i) {
      U8(static_cast<uint8_t>(v >> (8 * i)));
    }
  }

  void U64(uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      U8(static_cast<uint8_t>(v >> (8 * i)));
    }
  }

  void I16(int16_t v) { U16(static_cast<uint16_t>(v)); }
  void I32(int32_t v) { U32(static_cast<uint32_t>(v)); }

  void F32(float v) {
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    U32(bits);
  }

  void F64(double v) {
    uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    U64(bits);
  }

 private:
  std::string& out_;
};

// Little-endian reader over a byte span with bounds checking. Every read
// advances a cursor and sets an error flag if it would run past the end;
// callers check ok() once at the end rather than per field. After an error,
// further reads return zero, so parsing degrades safely.
class ByteReader {
 public:
  ByteReader(const char* data, size_t size) : data_(data), size_(size) {}

  bool ok() const { return ok_; }
  size_t remaining() const { return ok_ ? size_ - pos_ : 0; }

  uint8_t U8() {
    if (pos_ + 1 > size_) {
      ok_ = false;
      return 0;
    }
    return static_cast<uint8_t>(data_[pos_++]);
  }

  uint16_t U16() {
    uint16_t lo = U8();
    uint16_t hi = U8();
    return static_cast<uint16_t>(lo | (hi << 8));
  }

  uint32_t U32() {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
      v |= static_cast<uint32_t>(U8()) << (8 * i);
    }
    return v;
  }

  uint64_t U64() {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
      v |= static_cast<uint64_t>(U8()) << (8 * i);
    }
    return v;
  }

  int16_t I16() { return static_cast<int16_t>(U16()); }
  int32_t I32() { return static_cast<int32_t>(U32()); }

  float F32() {
    uint32_t bits = U32();
    float v = 0;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
  }

  double F64() {
    uint64_t bits = U64();
    double v = 0;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
  }

 private:
  const char* data_;
  size_t size_;
  size_t pos_ = 0;
  bool ok_ = true;
};

// A string pool that appends each string and returns a (offset, length)
// reference into a single blob. Deliberately does not deduplicate (the pool is
// only a few MB; simplicity wins).
class StringPool {
 public:
  std::pair<uint32_t, uint32_t> Add(const std::string& s) {
    const uint32_t offset = static_cast<uint32_t>(blob_.size());
    blob_.append(s);
    return {offset, static_cast<uint32_t>(s.size())};
  }

  const std::string& blob() const { return blob_; }

 private:
  std::string blob_;
};

// Resolve a (offset, len) reference against a loaded pool blob. Sets `ok` false
// and returns empty if the reference is out of range.
inline std::string ResolveRef(const std::string& blob, uint32_t offset, uint32_t len, bool& ok) {
  if (static_cast<size_t>(offset) + len > blob.size()) {
    ok = false;
    return {};
  }
  return blob.substr(offset, len);
}

}  // namespace bf
