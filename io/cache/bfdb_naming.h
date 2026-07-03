#pragma once

// Naming conventions for `.bfdb` graph caches, so a directory of caches from
// different AIRAC cycles can be discovered and told apart by filename alone.
//
// The canonical name encodes the cycle and build: `nav_<cycle>_<build>.bfdb`
// (e.g. nav_2601_20260112.bfdb). The companion CIFP procedure cache keeps the
// existing `<stem>_cifp.bfdb` derivation, so it becomes
// nav_<cycle>_<build>_cifp.bfdb automatically.
//
// IMPORTANT: the filename is only a discovery/display hint, never the source of
// truth. The authoritative cycle/build live in the cache header (see
// BfdbCache::ReadHeader); a caller must trust the header, not a name a user may
// have renamed. ParseBfdbName is therefore best-effort and used only to group
// and label files before opening them.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace bf {

// The canonical cache filename for a given cycle/build:
// "nav_<cycle>_<build>.bfdb". When both are zero (data with no parsed AIRAC
// provenance), falls back to the legacy "nav.bfdb" so a name always exists.
std::string FormatBfdbName(uint32_t cycle, uint32_t build);

// Best-effort parse of a "nav_<cycle>_<build>.bfdb" filename back into its
// (cycle, build). Returns nullopt when the name does not match the pattern --
// including the companion "*_cifp.bfdb" caches, which are deliberately rejected
// so a directory scan does not mistake them for graph caches. Accepts a bare
// filename or a path (only the filename component is inspected). This is a hint
// only; the header is authoritative.
std::optional<std::pair<uint32_t, uint32_t>> ParseBfdbName(std::string_view path);

}  // namespace bf
