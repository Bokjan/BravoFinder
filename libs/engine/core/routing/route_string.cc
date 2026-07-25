// SPDX-License-Identifier: LGPL-3.0-or-later
#include "core/routing/route_string.h"

#include <algorithm>
#include <string>
#include <vector>

namespace bf {

namespace {

// Keep the names in `a` that also appear in `b`, preserving a's order.
std::vector<std::string> Intersect(const std::vector<std::string>& a,
                                   const std::vector<std::string>& b) {
  std::vector<std::string> out;
  for (const std::string& s : a) {
    if (std::find(b.begin(), b.end(), s) != b.end()) {
      out.push_back(s);
    }
  }
  return out;
}

}  // namespace

// Split an airway designator field into its list of route names. A concurrency
// is encoded "A593-Y592"; a single airway is just "Y592"; "DCT" yields {"DCT"}.
// Real ATS route designators are letter+number with no internal hyphen, so '-'
// is an unambiguous separator. Source order is preserved (the sets are tiny --
// at most ~10 designators -- so a vector beats a tree here). Empty segments
// (a leading/trailing/double hyphen, or an empty input) are dropped, since an
// empty designator is never a valid airway and would otherwise be written into
// the route string and mistaken for a real name.
std::vector<std::string> SplitDesignators(const std::string& via) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= via.size()) {
    const size_t dash = via.find('-', start);
    if (dash == std::string::npos) {
      if (start < via.size()) {
        out.push_back(via.substr(start));
      }
      break;
    }
    if (dash > start) {
      out.push_back(via.substr(start, dash - start));
    }
    start = dash + 1;
  }
  return out;
}

std::string BuildRouteString(const std::string& first_point, std::vector<RouteLeg>& legs) {
  std::string rs = first_point;

  // Split every leg's via field exactly once up front, then index into this
  // below. Folding otherwise re-parses each group-boundary leg twice: once as
  // the `next` that breaks a group's running intersection, then again as the
  // via that opens the following group.
  std::vector<std::vector<std::string>> split(legs.size());
  for (size_t k = 0; k < legs.size(); ++k) {
    split[k] = SplitDesignators(legs[k].via);
  }

  size_t i = 0;
  while (i < legs.size()) {
    // A DCT leg never folds: it is a hard boundary emitted on its own.
    if (legs[i].via == "DCT") {
      legs[i].concurrent_airways.clear();
      rs += " DCT " + legs[i].to;
      ++i;
      continue;
    }

    // Grow a group of consecutive legs whose designator lists keep a non-empty
    // running intersection (i.e. they stay on a shared physical airway).
    std::vector<std::string> running = split[i];
    if (running.empty()) {
      // An empty via is not a real airway; treat the leg as DCT to stay safe.
      legs[i].concurrent_airways.clear();
      rs += " DCT " + legs[i].to;
      ++i;
      continue;
    }
    size_t j = i;
    while (j + 1 < legs.size() && legs[j + 1].via != "DCT") {
      std::vector<std::string> inter = Intersect(running, split[j + 1]);
      if (inter.empty()) {
        break;
      }
      running = std::move(inter);
      ++j;
    }

    // The chosen designator is valid on every leg in the group (it lies in the
    // running intersection, which is contained in each leg's list). Order is
    // irrelevant, so the first survivor is a fine deterministic pick.
    const std::string& chosen = running.front();
    for (size_t k = i; k <= j; ++k) {
      legs[k].via = chosen;
      if (split[k].size() > 1) {
        legs[k].concurrent_airways = std::move(split[k]);
      } else {
        legs[k].concurrent_airways.clear();
      }
    }

    rs += " " + chosen + " " + legs[j].to;
    i = j + 1;
  }

  return rs;
}

}  // namespace bf
