// SPDX-License-Identifier: MIT
#include "cli_common.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <charconv>
#include <cmath>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/base/string_util.h"

namespace bf::cli {

Result<NavDatabase> OpenForRead(const std::string& db_path, const std::string& data_dir,
                                const std::string& cifp_load) {
  if (db_path.empty()) {
    return NavDatabase::Open(data_dir);
  }
  return NavDatabase::OpenCached(db_path,
                                 cifp_load == "eager" ? CifpLoad::kEager : CifpLoad::kOnDemand);
}

std::optional<FlRange> ParseAltSpec(const std::string& spec) {
  const size_t dash = spec.find('-');
  auto to_int = [](const std::string& s, int& out) -> bool {
    if (s.empty()) {
      return false;
    }
    // std::from_chars is the exception-free counterpart of stoi: it fails via
    // an error code (no try/catch), and ptr == end verifies the whole field
    // was numeric. A leading '+' or whitespace is rejected, which is fine for
    // a flight-level spec.
    const char* begin = s.data();
    const char* end = begin + s.size();
    auto [ptr, ec] = std::from_chars(begin, end, out);
    return ec == std::errc{} && ptr == end && out >= 0;
  };
  if (dash == std::string::npos) {
    int fl = 0;
    if (!to_int(spec, fl)) {
      return std::nullopt;
    }
    return FlRange{fl, fl};
  }
  int lo = 0;
  int hi = 0;
  if (!to_int(spec.substr(0, dash), lo) || !to_int(spec.substr(dash + 1), hi)) {
    return std::nullopt;
  }
  if (lo > hi) {
    return std::nullopt;
  }
  return FlRange{lo, hi};
}

namespace {

// Split `field` on ',' and reject an empty element. An empty element almost always
// means a typo ("ZB,,ZG" or a trailing "ZB,"), and silently dropping it would turn
// the field into a match-everything rule -- the opposite of what the user wanted.
// Each entry is upper-cased: the navigation data stores region codes and
// designators upper-case, and the rest of the query layer already normalizes user
// input (see bf::ToUpper), so "zb,zg:j60" must mean the same as "ZB,ZG:J60".
bool SplitCsv(std::string_view field, std::vector<std::string>& out, std::string& error) {
  size_t start = 0;
  while (true) {
    const size_t comma = field.find(',', start);
    const std::string_view item = field.substr(
        start, comma == std::string_view::npos ? std::string_view::npos : comma - start);
    if (item.empty()) {
      error = "empty entry in '" + std::string(field) + "'";
      return false;
    }
    out.emplace_back(bf::ToUpper(std::string(item)));
    if (comma == std::string_view::npos) {
      return true;
    }
    start = comma + 1;
  }
}

// Strip the trailing-'*' markers from `items`, reporting via `starred` how many
// carried one. A lone "*" means match-everything, signalled by clearing the list.
//
// The marker is only meaningful on the designator side, where it selects the match
// mode; on the region side it is cosmetic (regions are always prefix-matched), so
// the caller decides whether a mixed count is an error.
bool StripStarMarkers(std::vector<std::string>& items, size_t& starred, std::string& error) {
  starred = 0;
  if (items.size() == 1 && items[0] == "*") {
    items.clear();  // empty list == match everything
    return true;
  }
  for (std::string& item : items) {
    if (item.back() == '*') {
      item.pop_back();
      ++starred;
      if (item.empty()) {
        error = "'*' cannot be combined with other entries; use a bare '*' alone";
        return false;
      }
    }
  }
  return true;
}

}  // namespace

std::optional<AirwayRule> ParseAirwayFilter(const std::string& spec, std::string& error) {
  // Fields never contain ':' or '=' themselves (region codes and airway designators
  // are alphanumeric), so splitting on the FIRST occurrence of each is unambiguous.
  const size_t colon = spec.find(':');
  if (colon == std::string::npos) {
    error = "expected '<regions>:<designators>[=<action>[:<fraction>]]'";
    return std::nullopt;
  }
  const std::string_view regions_field(spec.data(), colon);
  std::string_view rest(spec.data() + colon + 1, spec.size() - colon - 1);
  std::string_view action_field;
  const size_t equals = rest.find('=');
  if (equals != std::string_view::npos) {
    action_field = rest.substr(equals + 1);
    rest = rest.substr(0, equals);
  }
  if (regions_field.empty() || rest.empty()) {
    error = "region and designator fields must not be empty (use '*' for 'any')";
    return std::nullopt;
  }

  AirwayRule rule;
  if (!SplitCsv(regions_field, rule.region_prefixes, error)) {
    return std::nullopt;
  }
  // Regions are prefix-matched unconditionally, so their '*' markers carry no
  // meaning beyond readability -- "ZB*,ZG" and "ZB,ZG" are the same rule. Strip
  // them and ignore the count (unlike designators, a mix is not an error).
  size_t region_stars = 0;
  if (!StripStarMarkers(rule.region_prefixes, region_stars, error)) {
    return std::nullopt;
  }
  if (!SplitCsv(rest, rule.designators, error)) {
    return std::nullopt;
  }
  size_t designator_stars = 0;
  if (!StripStarMarkers(rule.designators, designator_stars, error)) {
    return std::nullopt;
  }
  if (rule.designators.empty()) {
    // A bare "*" cleared the list, meaning "every designator". The match mode is
    // then irrelevant; keep it at kPrefix so the value is never a stale kExact.
    rule.match = AirwayRule::Match::kPrefix;
  } else if (designator_stars == 0) {
    rule.match = AirwayRule::Match::kExact;
  } else if (designator_stars == rule.designators.size()) {
    rule.match = AirwayRule::Match::kPrefix;
  } else {
    // The match mode is per rule, so a mix has no single correct reading. Reject it
    // rather than guessing, and point at the fix.
    error =
        "cannot mix prefix ('J*') and exact ('J60') designators in one filter; "
        "pass them as two --airway-filter values";
    return std::nullopt;
  }

  if (action_field.empty()) {
    return rule;  // no action given: penalize at the default fraction
  }
  std::string_view action = action_field;
  std::string_view value;
  const size_t value_sep = action_field.find(':');
  if (value_sep != std::string_view::npos) {
    action = action_field.substr(0, value_sep);
    value = action_field.substr(value_sep + 1);
  }
  if (action == "block") {
    // Reject "block:0.5" rather than ignoring the number: a silently dropped value
    // reads as if the penalty took effect.
    if (!value.empty()) {
      error = "'block' takes no value (drop the ':" + std::string(value) + "')";
      return std::nullopt;
    }
    rule.action = AirwayRule::Action::kBlock;
    return rule;
  }
  if (action != "penalize") {
    error = "unknown action '" + std::string(action) + "' (expected 'block' or 'penalize')";
    return std::nullopt;
  }
  rule.action = AirwayRule::Action::kPenalize;
  if (value.empty()) {
    return rule;  // penalize at the default fraction
  }
  double fraction = 0.0;
  const char* fbegin = value.data();
  const char* fend = fbegin + value.size();
  auto [fptr, fec] = std::from_chars(fbegin, fend, fraction);
  if (fec != std::errc{} || fptr != fend) {
    error = "penalty fraction '" + std::string(value) + "' must be a number >= 0";
    return std::nullopt;
  }
  // Reject negatives and non-finite values: a negative penalty would break the
  // search heuristic's admissibility. No upper bound -- a large fraction is a
  // legitimate "almost block, but keep the graph connected".
  if (!std::isfinite(fraction) || fraction < 0.0) {
    error = "penalty fraction '" + std::string(value) + "' must be a number >= 0";
    return std::nullopt;
  }
  rule.penalty_fraction = fraction;
  return rule;
}

std::string WrapRoutesEnvelope(const std::string& routes_body, uint32_t elapsed_ms) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("routes");
  // routes_body is already valid RapidJSON output from the query layer; splice
  // it in verbatim rather than re-parsing it.
  writer.RawValue(routes_body.data(), routes_body.size(), rapidjson::kArrayType);
  writer.Key("elapsed_ms");
  writer.Uint(elapsed_ms);
  writer.EndObject();
  return buffer.GetString();
}

std::string WrapRouteEnvelope(const std::string& route_body, uint32_t elapsed_ms) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("route");
  // route_body is already valid RapidJSON output from the query layer (a single
  // route object for parse_route); splice it in verbatim rather than re-parsing.
  writer.RawValue(route_body.data(), route_body.size(), rapidjson::kObjectType);
  writer.Key("elapsed_ms");
  writer.Uint(elapsed_ms);
  writer.EndObject();
  return buffer.GetString();
}

}  // namespace bf::cli
