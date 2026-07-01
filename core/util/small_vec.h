#pragma once

#include <cstddef>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <utility>

namespace bf {

// Inline capacity for the `ident -> regions` map in GraphBuilder. Derived from a
// measured AIRAC 2601 distribution: 98.88% of idents are reused across <= 4
// regions, so N=4 keeps the inline storage covering everything but a 1.12%
// long tail that spills to the heap once, at build time (cost negligible). Do
// NOT reuse this constant for other map-of-vector members without measuring
// their own distribution -- N is per-site, not a global default.
inline constexpr int kIdentRegionInline = 4;

// A tiny vector that stores up to N elements inline and only allocates on the
// heap when it grows past N. Used to back map-of-small-vector indices without
// paying a heap allocation for the common (small) case. Intentionally minimal:
// it only supports the operations GraphBuilder needs (push_back, indexed/size
// access, range iteration, move). Zero external dependencies, per the project's
// dependency discipline (no Abseil/Boost/LLVM).
template <typename T, int N>
class SmallVec {
  static_assert(N > 0, "SmallVec inline capacity must be positive");

 public:
  SmallVec() = default;

  SmallVec(std::initializer_list<T> il) {
    for (const T& x : il) {
      push_back(x);
    }
  }

  // Copy is deep (the heap buffer, if any, is duplicated).
  SmallVec(const SmallVec& other) : size_(other.size_) {
    if (other.uses_inline()) {
      for (size_t i = 0; i < size_; ++i) {
        inline_[i] = other.inline_[i];
      }
    } else {
      data_ = Allocate(other.capacity_);
      capacity_ = other.capacity_;
      for (size_t i = 0; i < size_; ++i) {
        data_[i] = other.data_[i];
      }
    }
  }

  SmallVec& operator=(const SmallVec& other) {
    if (this != &other) {
      SmallVec tmp(other);
      *this = std::move(tmp);
    }
    return *this;
  }

  // Move leaves the source empty and inline (it owns nothing to free).
  SmallVec(SmallVec&& other) noexcept
      : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
    if (other.uses_inline()) {
      for (size_t i = 0; i < size_; ++i) {
        inline_[i] = other.inline_[i];
      }
      data_ = inline_;
    } else {
      other.data_ = nullptr;
    }
    other.size_ = 0;
    other.capacity_ = 0;
  }

  SmallVec& operator=(SmallVec&& other) noexcept {
    if (this != &other) {
      FreeIfHeap();
      size_ = other.size_;
      capacity_ = other.capacity_;
      data_ = other.data_;
      if (other.uses_inline()) {
        for (size_t i = 0; i < size_; ++i) {
          inline_[i] = other.inline_[i];
        }
        data_ = inline_;
      } else {
        other.data_ = nullptr;
      }
      other.size_ = 0;
      other.capacity_ = 0;
    }
    return *this;
  }

  ~SmallVec() { FreeIfHeap(); }

  void push_back(const T& value) {
    if (size_ == capacity_) {
      Grow();
    }
    data_[size_++] = value;
  }

  void push_back(T&& value) {
    if (size_ == capacity_) {
      Grow();
    }
    data_[size_++] = std::move(value);
  }

  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }

  T& operator[](size_t i) { return data_[i]; }
  const T& operator[](size_t i) const { return data_[i]; }

  T& front() { return data_[0]; }
  const T& front() const { return data_[0]; }
  T& back() { return data_[size_ - 1]; }
  const T& back() const { return data_[size_ - 1]; }

  // Range iteration over the live elements.
  const T* begin() const { return data_; }
  const T* end() const { return data_ + size_; }
  T* begin() { return data_; }
  T* end() { return data_ + size_; }

 private:
  bool uses_inline() const { return data_ == inline_; }

  static T* Allocate(size_t n) {
    // align to T; malloc gives suitable alignment for the element type.
    return static_cast<T*>(std::malloc(n * sizeof(T)));
  }

  void FreeIfHeap() {
    if (!uses_inline() && data_ != nullptr) {
      std::free(data_);
    }
  }

  // Grow: move inline (or current heap) contents into a doubled heap buffer once
  // past N. The inline path is taken exactly once, at the N->N+1 transition.
  void Grow() {
    const size_t new_cap = capacity_ == 0 ? N + 1 : capacity_ * 2;
    T* next = Allocate(new_cap);
    for (size_t i = 0; i < size_; ++i) {
      next[i] = std::move(data_[i]);
    }
    FreeIfHeap();
    data_ = next;
    capacity_ = new_cap;
  }

  T inline_[N];
  T* data_ = inline_;
  size_t size_ = 0;
  size_t capacity_ = N;
};

}  // namespace bf
