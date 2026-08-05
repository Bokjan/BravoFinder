// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

namespace bf {

// Shared sentinels for graph search and route metadata. Named so call sites
// cannot confuse "no vertex / no bearing" with a legitimate zero index or a
// zero heading (0° is north and is a real bearing). Lives in core/domain (not
// core/graph) because routing/route.h needs it for approach metadata without
// depending on the graph directory.

inline constexpr int kNoVertex = -1;
inline constexpr double kNoBearing = -1.0;

}  // namespace bf
