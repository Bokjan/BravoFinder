#include "core/routing/route_string.h"

#include <set>
#include <string>
#include <vector>

namespace bf {

namespace {

// Split an airway designator field into its set of route names. A concurrency
// is encoded "A593-Y592"; a single airway is just "Y592"; "DCT" yields {"DCT"}.
// Real ATS route designators are letter+number with no internal hyphen, so '-'
// is an unambiguous separator.
std::set<std::string> SplitDesignators(const std::string& via) {
  std::set<std::string> out;
  size_t start = 0;
  while (start <= via.size()) {
    const size_t dash = via.find('-', start);
    if (dash == std::string::npos) {
      out.insert(via.substr(start));
      break;
    }
    out.insert(via.substr(start, dash - start));
    start = dash + 1;
  }
  return out;
}

std::set<std::string> Intersect(const std::set<std::string>& a, const std::set<std::string>& b) {
  std::set<std::string> out;
  for (const std::string& s : a) {
    if (b.count(s) != 0) {
      out.insert(s);
    }
  }
  return out;
}

}  // namespace

std::string BuildRouteString(const std::string& first_point, std::vector<RouteLeg>& legs) {
  std::string rs = first_point;

  size_t i = 0;
  while (i < legs.size()) {
    // A DCT leg never folds: it is a hard boundary emitted on its own.
    if (legs[i].via == "DCT") {
      legs[i].concurrent_airways.clear();
      rs += " DCT " + legs[i].to;
      ++i;
      continue;
    }

    // Grow a group of consecutive legs whose designator sets keep a non-empty
    // running intersection (i.e. they stay on a shared physical airway).
    std::set<std::string> running = SplitDesignators(legs[i].via);
    size_t j = i;
    while (j + 1 < legs.size() && legs[j + 1].via != "DCT") {
      const std::set<std::string> next = SplitDesignators(legs[j + 1].via);
      const std::set<std::string> inter = Intersect(running, next);
      if (inter.empty()) {
        break;
      }
      running = inter;
      ++j;
    }

    // The chosen designator is valid on every leg in the group (it lies in the
    // running intersection, which is contained in each leg's set); the smallest
    // survivor gives a deterministic pick when the group never narrows to one.
    const std::string& chosen = *running.begin();
    for (size_t k = i; k <= j; ++k) {
      const std::set<std::string> set = SplitDesignators(legs[k].via);
      legs[k].via = chosen;
      if (set.size() > 1) {
        legs[k].concurrent_airways.assign(set.begin(), set.end());
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
