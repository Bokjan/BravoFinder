// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Internal little-endian byte serialization helpers shared by the cache
// section codecs (graph_codec, cifp_codec, nav_detail_codec). Not a public API.
//
// Integers are emitted least-significant-byte first, so output is identical
// regardless of host endianness. Floats are written via their IEEE-754 bit
// pattern (memcpy to an unsigned integer of the same width), which every
// current platform shares, so values round-trip exactly across architectures.
//
// Byte buffers are std::vector<uint8_t> / std::span<const uint8_t> -- these hold
// raw bytes, not text. Decoded strings (idents, regions, ICAO codes) are still
// std::string, since those ARE text resolved out of the shared string pool.

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bf {

// Little-endian writer appending to an in-memory byte buffer.
class ByteWriter {
 public:
  explicit ByteWriter(std::vector<uint8_t>& out) : out_(out) {}

  // Grow the backing buffer's capacity to at least its current size plus `extra`
  // bytes, so a sequence of appends does not reallocate. A hint only; callers
  // estimate the total from element counts before writing the sections.
  void Reserve(size_t extra) { out_.reserve(out_.size() + extra); }

  void U8(uint8_t v) { out_.push_back(v); }

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

  // Append `n` raw bytes verbatim (no endianness transform). Used for string
  // bytes and pre-serialized blobs.
  void Bytes(const uint8_t* data, size_t n) { out_.insert(out_.end(), data, data + n); }

  // A length-prefixed string: U32 length, then the bytes. The inverse of
  // ByteReader::Str. Used for inline header strings (provenance, etc.) and for
  // pool-interned string bodies. The input is text; it is serialized as bytes.
  void Str(const std::string& s) {
    U32(static_cast<uint32_t>(s.size()));
    Bytes(reinterpret_cast<const uint8_t*>(s.data()), s.size());
  }

 private:
  std::vector<uint8_t>& out_;
};

// Little-endian reader over a byte span with bounds checking. Every read
// advances a cursor and sets an error flag if it would run past the end;
// callers check ok() once at the end rather than per field. After an error,
// further reads return zero, so parsing degrades safely.
class ByteReader {
 public:
  ByteReader(std::span<const uint8_t> bytes) : data_(bytes.data()), size_(bytes.size()) {}

  bool ok() const { return ok_; }
  size_t remaining() const { return ok_ ? size_ - pos_ : 0; }

  uint8_t U8() {
    if (pos_ + 1 > size_) {
      ok_ = false;
      return 0;
    }
    return data_[pos_++];
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

  // Copy `n` raw bytes into `dst` (no endianness transform). Sets the error
  // flag and copies nothing if fewer than `n` bytes remain.
  void Bytes(uint8_t* dst, size_t n) {
    if (pos_ + n > size_) {
      ok_ = false;
      return;
    }
    std::memcpy(dst, data_ + pos_, n);
    pos_ += n;
  }

  // Read a length-prefixed string (U32 length, then bytes). The inverse of
  // ByteWriter::Str. On truncation, sets the error flag and returns empty.
  std::string Str() {
    const uint32_t len = U32();
    if (!ok_ || len > remaining()) {
      ok_ = false;
      return {};
    }
    std::string s(len, '\0');
    std::memcpy(s.data(), data_ + pos_, len);
    pos_ += len;
    return s;
  }

  // Bulk-decode `n` little-endian int16 values into `dst`. On a little-endian
  // host this is a single memcpy; on big-endian it byte-swaps each element.
  // Equivalent to calling I16() n times but avoids the per-element call
  // overhead for large runs (e.g. the 64800-cell MORA grid).
  void I16Span(int16_t* dst, size_t n) {
    if (pos_ + n * sizeof(int16_t) > size_) {
      ok_ = false;
      return;
    }
    std::memcpy(dst, data_ + pos_, n * sizeof(int16_t));
    pos_ += n * sizeof(int16_t);
    if constexpr (std::endian::native == std::endian::big) {
      for (size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<int16_t>(ByteSwap16(static_cast<uint16_t>(dst[i])));
      }
    }
  }

  // Bulk-decode `n` little-endian int32 values into `dst`. Like I16Span, a
  // single memcpy on a little-endian host. Used for plain int32 arrays such as
  // the CSR offsets row.
  void I32Span(int32_t* dst, size_t n) {
    if (pos_ + n * sizeof(int32_t) > size_) {
      ok_ = false;
      return;
    }
    std::memcpy(dst, data_ + pos_, n * sizeof(int32_t));
    pos_ += n * sizeof(int32_t);
    if constexpr (std::endian::native == std::endian::big) {
      for (size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<int32_t>(ByteSwap32(static_cast<uint32_t>(dst[i])));
      }
    }
  }

 private:
  static uint16_t ByteSwap16(uint16_t v) { return static_cast<uint16_t>((v << 8) | (v >> 8)); }
  static uint32_t ByteSwap32(uint32_t v) {
    return ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) | ((v & 0x00FF0000u) >> 8) |
           ((v & 0xFF000000u) >> 24);
  }

  const uint8_t* data_;
  size_t size_;
  size_t pos_ = 0;
  bool ok_ = true;
};

// A string pool that appends each distinct string once and returns a (offset,
// length) reference into a single blob. Repeated strings (idents, regions,
// airway names, ICAO codes recur heavily) resolve to the same reference, so the
// blob holds one copy of each. Deduplication is transparent to readers: the
// on-disk reference format is unchanged, so a pool written with or without it
// reads identically. The blob is a byte buffer (the serialized pool), while the
// dedup map keys are the text strings being interned.
class StringPool {
 public:
  std::pair<uint32_t, uint32_t> Add(const std::string& s) {
    auto [it, inserted] = interned_.try_emplace(s, std::pair<uint32_t, uint32_t>{});
    if (!inserted) {
      return it->second;
    }
    const std::pair<uint32_t, uint32_t> ref{static_cast<uint32_t>(blob_.size()),
                                            static_cast<uint32_t>(s.size())};
    blob_.insert(blob_.end(), reinterpret_cast<const uint8_t*>(s.data()),
                 reinterpret_cast<const uint8_t*>(s.data()) + s.size());
    it->second = ref;
    return ref;
  }

  std::span<const uint8_t> blob() const { return blob_; }

 private:
  std::vector<uint8_t> blob_;
  std::unordered_map<std::string, std::pair<uint32_t, uint32_t>> interned_;
};

// Resolve a (offset, len) reference against a loaded pool blob. Sets `ok` false
// and returns empty if the reference is out of range. The resolved value is
// text (an ident, region, or ICAO code), hence std::string.
inline std::string ResolveRef(std::span<const uint8_t> pool, uint32_t offset, uint32_t len,
                              bool& ok) {
  // Overflow-safe bounds check: offset + len can wrap when size_t is 32-bit, so
  // subtract instead of add. offset <= size is checked first, so size - offset
  // never underflows.
  if (offset > pool.size() || len > pool.size() - offset) {
    ok = false;
    return {};
  }
  return std::string(reinterpret_cast<const char*>(pool.data() + offset), len);
}

}  // namespace bf
