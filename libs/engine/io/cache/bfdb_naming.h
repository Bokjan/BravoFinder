// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

// Naming conventions for `.bfdb` caches, so a directory of caches from different
// AIRAC cycles can be discovered and told apart by filename alone.
//
// The canonical name encodes the cycle: `nav_<cycle>.bfdb` (e.g. nav_2601.bfdb).
// A unified `.bfdb` holds graph + CIFP + detail in one file, so there are no
// longer any companion files to distinguish.
//
// IMPORTANT: the filename is only a discovery/display hint, never the source of
// truth. The authoritative cycle lives in the cache header (see
// UnifiedCache::ReadHeader); a caller must trust the header, not a name a user
// may have renamed. ParseBfdbName is therefore best-effort and used only to
// group and label files before opening them.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace bf {

// The canonical cache filename for a given cycle: "nav_<cycle>.bfdb". When the
// cycle is zero (data with no parsed AIRAC provenance), falls back to the legacy
// "nav.bfdb" so a name always exists.
std::string FormatBfdbName(uint32_t cycle);

// Best-effort parse of a "nav_<cycle>.bfdb" filename back into its cycle.
// Returns nullopt when the name does not match the pattern. The legacy
// "nav.bfdb" (the zero-cycle sentinel produced by FormatBfdbName(0)) parses back
// to 0, so the two functions round-trip. Accepts a bare filename or a path (only
// the filename component is inspected). This is a hint only; the header is
// authoritative.
std::optional<uint32_t> ParseBfdbName(std::string_view path);

}  // namespace bf
