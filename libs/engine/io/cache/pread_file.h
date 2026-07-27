// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// A minimal, cross-platform read-only file handle supporting positional reads
// (read at an explicit offset without a shared file cursor). Used by CifpArchive
// to fetch on-disk segments concurrently from multiple threads: pread (POSIX)
// and ReadFile with an OVERLAPPED offset (Windows) both leave the handle's
// position untouched, so parallel ReadAt calls on one PreadFile share no mutable
// state and are race-free -- satisfying NavDatabase contract B without a lock.
//
// Move-only: it owns the underlying descriptor/handle and closes it on
// destruction. A default-constructed or moved-from instance is not is_open().

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>

#ifdef _WIN32
// Suppress windows.h's min/max function-like macros (they collide with
// std::min/std::max in headers that transitively include this one) and trim the
// header to speed compilation.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace bf {

class PreadFile {
 public:
  PreadFile() = default;

  // Open `path` read-only. Check is_open() afterwards.
  explicit PreadFile(const std::string& path) {
#ifdef _WIN32
    handle_ = ::CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING,
                            FILE_ATTRIBUTE_NORMAL, nullptr);
#else
    fd_ = ::open(path.c_str(), O_RDONLY);
#endif
  }

  ~PreadFile() { Close(); }

  PreadFile(const PreadFile&) = delete;
  PreadFile& operator=(const PreadFile&) = delete;

  PreadFile(PreadFile&& other) noexcept { *this = std::move(other); }

  PreadFile& operator=(PreadFile&& other) noexcept {
    if (this != &other) {
      Close();
#ifdef _WIN32
      handle_ = other.handle_;
      other.handle_ = INVALID_HANDLE_VALUE;
#else
      fd_ = other.fd_;
      other.fd_ = -1;
#endif
    }
    return *this;
  }

#ifdef _WIN32
  bool is_open() const { return handle_ != INVALID_HANDLE_VALUE; }
#else
  bool is_open() const { return fd_ >= 0; }
#endif

  // Read exactly `dst.size()` bytes at absolute `offset` into `dst`. Returns true
  // only if all bytes were read. Does not move any shared file position, so
  // concurrent calls on the same PreadFile are safe.
  bool ReadAt(std::span<uint8_t> dst, uint64_t offset) const {
    char* const base = reinterpret_cast<char*>(dst.data());
    const size_t len = dst.size();
    size_t got = 0;
    while (got < len) {
#ifdef _WIN32
      OVERLAPPED ov{};
      const uint64_t pos = offset + got;
      ov.Offset = static_cast<DWORD>(pos & 0xFFFFFFFFu);
      ov.OffsetHigh = static_cast<DWORD>(pos >> 32);
      DWORD read = 0;
      const DWORD want = static_cast<DWORD>((len - got) > 0xFFFFFFFFu ? 0xFFFFFFFFu : (len - got));
      if (!::ReadFile(handle_, base + got, want, &read, &ov) || read == 0) {
        return false;
      }
      got += read;
#else
      const ssize_t n = ::pread(fd_, base + got, len - got, static_cast<off_t>(offset + got));
      if (n <= 0) {
        return false;
      }
      got += static_cast<size_t>(n);
#endif
    }
    return true;
  }

 private:
  void Close() {
#ifdef _WIN32
    if (handle_ != INVALID_HANDLE_VALUE) {
      ::CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
    }
#else
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
#endif
  }

#ifdef _WIN32
  HANDLE handle_ = INVALID_HANDLE_VALUE;
#else
  int fd_ = -1;
#endif
};

}  // namespace bf
