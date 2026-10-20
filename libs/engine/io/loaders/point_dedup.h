// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string_view>
#include <unordered_map>
#include <vector>

#include "core/base/log.h"
#include "core/domain/coordinate.h"
#include "core/domain/fixed_string.h"
#include "core/domain/ident.h"

namespace bf {

// Two same-key source rows closer than this are the same physical point (a
// VOR-DME listed as both a VOR and a DME, a fix listed in both the enroute and
// terminal tables) and become one vertex. Distinct same-key points -- e.g. the
// DER06/LF departure-end fix of each French airport with a runway 06 -- are
// tens to hundreds of NM apart and are all kept. Cycle 2601 shows a clean gap:
// true duplicates sit within ~0.3 NM, distinct points are >= ~1 NM apart.
inline constexpr double kSamePointNm = 0.5;

// First-wins merge of same-key points by position, shared by the loaders. An
// (ident, region) key alone is not unique, so deduplicating on the key (the
// former behavior) silently dropped every later point that reused it.
class PointDeduper {
 public:
  explicit PointDeduper(std::string_view loader_name) : loader_name_(loader_name) {}

  // Record `coord` under `key` and return true if the row is a new routable
  // point: its key fits the graph's compact FixedIdent (GraphBuilder rejects
  // the whole build on an oversized key, so such a row is skipped with a
  // warning) and no point with the same key was already recorded within
  // kSamePointNm. Return false (record nothing) otherwise.
  bool Insert(const Ident& key, const Coordinate& coord) {
    if (!FixedIdent::FromIdent(key)) {
      BF_LOG_WARN("{}: skipping point '{}/{}' (key too long for FixedIdent)", loader_name_,
                  key.ident, key.arinc424_icao_code);
      return false;
    }
    std::vector<Coordinate>& points = seen_[key];
    for (const Coordinate& p : points) {
      if (p.DistanceTo(coord) < kSamePointNm) {
        return false;
      }
    }
    points.push_back(coord);
    return true;
  }

 private:
  std::string_view loader_name_;
  std::unordered_map<Ident, std::vector<Coordinate>> seen_;
};

}  // namespace bf
