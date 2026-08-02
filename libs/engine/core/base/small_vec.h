// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <cstddef>
#include <cstdlib>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

namespace bf {

// A tiny vector that stores up to N elements inline and only allocates on the
// heap when it grows past N. A general-purpose utility (currently exercised only
// by its own unit test after the GraphBuilder ident-map migration); use it to
// back map-of-small-vector indices without paying a heap allocation for the
// common (small) case. Intentionally minimal: push_back, indexed/size access,
// range iteration, move. Zero external dependencies, per the project's
// dependency discipline (no Abseil/Boost/LLVM).
//
// Restricted to trivial element types: the heap buffer is raw malloc storage
// that elements are assigned into (never placement-new constructed), and the
// inline array is default-initialized, so a non-trivial T would assign onto
// unconstructed memory and never run destructors -- undefined behavior. The
// static_assert enforces this; lifting it would mean adding real construct/
// destroy plumbing, which this minimal type deliberately avoids.
template <typename T, int N>
class SmallVec {
  static_assert(N > 0, "SmallVec inline capacity must be positive");
  static_assert(std::is_trivial_v<T>,
                "SmallVec only supports trivial element types (it assigns into "
                "raw malloc storage without constructing/destroying elements)");

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
    }
    // Leave the source a valid empty inline vector (data_ -> inline_, capacity N)
    // so a later push_back keeps its small-buffer optimization instead of heap-
    // allocating from a capacity of 0. In the heap case this also detaches the
    // source from the buffer `this` now owns, so its destructor won't free it.
    other.data_ = other.inline_;
    other.size_ = 0;
    other.capacity_ = N;
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
      }
      // Reset the source to a valid empty inline vector (see the move ctor).
      other.data_ = other.inline_;
      other.size_ = 0;
      other.capacity_ = N;
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
    T* p = static_cast<T*>(std::malloc(n * sizeof(T)));
    if (p == nullptr) {
      throw std::bad_alloc();  // OOM is exceptional; mirror operator new
    }
    return p;
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
