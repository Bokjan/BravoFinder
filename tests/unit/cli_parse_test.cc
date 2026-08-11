// SPDX-License-Identifier: MIT
// Unit tests for the CLI argument-spec parsers -- the string syntax that
// deserializes into the engine's structured request fields. The engine never sees
// these strings (the routing layer takes AirwayRule values), so this is where the
// spelling rules are pinned.
#include <catch2/catch_test_macros.hpp>
#include <string>

#include "cli_common.h"

namespace {

using Match = bf::AirwayRule::Match;
using Action = bf::AirwayRule::Action;

// Parse a spec that is expected to succeed, failing the test with the parser's own
// message otherwise.
bf::AirwayRule Ok(const std::string& spec) {
  std::string error;
  std::optional<bf::AirwayRule> rule = bf::cli::ParseAirwayFilter(spec, error);
  REQUIRE(rule.has_value());
  CHECK(error.empty());
  return *rule;
}

// Assert a spec is rejected, and return the message so the test can check it names
// the actual problem (a generic "invalid" would pass a weaker assertion).
std::string Err(const std::string& spec) {
  std::string error;
  std::optional<bf::AirwayRule> rule = bf::cli::ParseAirwayFilter(spec, error);
  CHECK_FALSE(rule.has_value());
  CHECK_FALSE(error.empty());
  return error;
}

TEST_CASE("airway filter: region and designator lists", "[unit][cli]") {
  // The ten mainland-China FIRs in one rule: all entries share the rule, so this
  // is one rule and not ten.
  const bf::AirwayRule china = Ok("ZB,ZG,ZH,ZJ,ZL,ZP,ZS,ZU,ZW,ZY:J*=block");
  CHECK(china.region_prefixes.size() == 10);
  CHECK(china.region_prefixes.front() == "ZB");
  CHECK(china.region_prefixes.back() == "ZY");
  CHECK(china.designators == std::vector<std::string>{"J"});
  CHECK(china.match == Match::kPrefix);
  CHECK(china.action == Action::kBlock);

  // Several exact designators, region-restricted.
  const bf::AirwayRule multi = Ok("ZB,ZG:J60,A3=block");
  CHECK(multi.region_prefixes == std::vector<std::string>{"ZB", "ZG"});
  CHECK(multi.designators == std::vector<std::string>{"J60", "A3"});
  CHECK(multi.match == Match::kExact);
}

TEST_CASE("airway filter: a bare '*' means any", "[unit][cli]") {
  // An empty list is how the rule spells "match everything" on that side.
  const bf::AirwayRule any_region = Ok("*:V*=penalize:0.8");
  CHECK(any_region.region_prefixes.empty());
  CHECK(any_region.designators == std::vector<std::string>{"V"});
  CHECK(any_region.match == Match::kPrefix);

  const bf::AirwayRule any_designator = Ok("ZS:*=block");
  CHECK(any_designator.region_prefixes == std::vector<std::string>{"ZS"});
  CHECK(any_designator.designators.empty());
  // With no designators to compare, the mode is irrelevant but must not be a stale
  // kExact -- it stays kPrefix so the value reads sensibly.
  CHECK(any_designator.match == Match::kPrefix);

  const bf::AirwayRule any_both = Ok("*:*=penalize");
  CHECK(any_both.region_prefixes.empty());
  CHECK(any_both.designators.empty());
}

TEST_CASE("airway filter: trailing '*' selects prefix vs exact matching", "[unit][cli]") {
  // This distinction is the whole reason for the marker: 1371 designators are a
  // strict prefix of another one, so exact J60 must not become prefix J60.
  CHECK(Ok("*:J60=block").match == Match::kExact);
  CHECK(Ok("*:J60*=block").match == Match::kPrefix);
  CHECK(Ok("*:J60").designators == std::vector<std::string>{"J60"});
  CHECK(Ok("*:J60*").designators == std::vector<std::string>{"J60"});  // marker stripped
}

TEST_CASE("airway filter: region '*' markers are cosmetic", "[unit][cli]") {
  // Regions are always prefix-matched (a code is at most two chars), so "Z" and
  // "Z*" mean the same thing and the marker is only stripped.
  CHECK(Ok("Z:J*").region_prefixes == std::vector<std::string>{"Z"});
  CHECK(Ok("Z*:J*").region_prefixes == std::vector<std::string>{"Z"});
  CHECK(Ok("ZB*,ZG:J*").region_prefixes == std::vector<std::string>{"ZB", "ZG"});
}

TEST_CASE("airway filter: action and fraction defaults", "[unit][cli]") {
  // No action given: penalize at the documented 0.5 default.
  const bf::AirwayRule bare = Ok("Z*:J*");
  CHECK(bare.action == Action::kPenalize);
  CHECK(bare.penalty_fraction == 0.5);

  // penalize with no value: same default.
  const bf::AirwayRule penalize = Ok("Z*:J*=penalize");
  CHECK(penalize.action == Action::kPenalize);
  CHECK(penalize.penalty_fraction == 0.5);

  const bf::AirwayRule explicit_fraction = Ok("Z*:J*=penalize:0.3");
  CHECK(explicit_fraction.penalty_fraction == 0.3);

  // No upper bound: a large fraction is a legitimate "almost block, but keep the
  // graph connected".
  CHECK(Ok("Z*:J*=penalize:100").penalty_fraction == 100.0);
  // Zero is a valid no-op penalty rather than an error.
  CHECK(Ok("Z*:J*=penalize:0").penalty_fraction == 0.0);
}

TEST_CASE("airway filter: malformed specs are rejected with a specific reason", "[unit][cli]") {
  // Missing the ':' separator entirely.
  CHECK(Err("ZS").find("<regions>:<designators>") != std::string::npos);
  // Empty field on either side: '*' is the way to say "any", so an empty field is
  // a typo rather than a shorthand.
  CHECK(Err(":J*").find("must not be empty") != std::string::npos);
  CHECK(Err("ZS:").find("must not be empty") != std::string::npos);
  // An empty list entry would silently widen the rule to match-everything.
  CHECK(Err("ZB,,ZG:J*").find("empty entry") != std::string::npos);
  CHECK(Err("ZB,:J*").find("empty entry") != std::string::npos);
  CHECK(Err("ZS:J*,=block").find("empty entry") != std::string::npos);
  // Mixing prefix and exact designators: the match mode is per rule, so this is
  // rejected rather than guessed at.
  CHECK(Err("ZS:J*,A3=block").find("cannot mix prefix") != std::string::npos);
  // A value on 'block' would be silently ignored, reading as if it applied.
  CHECK(Err("ZS:J*=block:0.5").find("takes no value") != std::string::npos);
  // A negative penalty would break the search heuristic's admissibility.
  CHECK(Err("ZS:J*=penalize:-1").find(">= 0") != std::string::npos);
  CHECK(Err("ZS:J*=penalize:abc").find("must be a number") != std::string::npos);
  CHECK(Err("ZS:J*=frobnicate").find("unknown action") != std::string::npos);
}

TEST_CASE("alt spec: single level and range", "[unit][cli]") {
  // Kept alongside the airway-filter cases since both are CLI spec parsers.
  const std::optional<bf::FlRange> single = bf::cli::ParseAltSpec("350");
  REQUIRE(single.has_value());
  CHECK(single->min_fl == 350);
  CHECK(single->max_fl == 350);

  const std::optional<bf::FlRange> range = bf::cli::ParseAltSpec("300-400");
  REQUIRE(range.has_value());
  CHECK(range->min_fl == 300);
  CHECK(range->max_fl == 400);

  CHECK_FALSE(bf::cli::ParseAltSpec("400-300").has_value());  // inverted
  CHECK_FALSE(bf::cli::ParseAltSpec("").has_value());
  CHECK_FALSE(bf::cli::ParseAltSpec("FL350").has_value());
  CHECK_FALSE(bf::cli::ParseAltSpec("350x").has_value());
  // Help text promises 0..kMaxFl; anything above that band must be rejected.
  CHECK_FALSE(bf::cli::ParseAltSpec("601").has_value());
  CHECK_FALSE(bf::cli::ParseAltSpec("300-601").has_value());
  const std::optional<bf::FlRange> at_max = bf::cli::ParseAltSpec("600");
  REQUIRE(at_max.has_value());
  CHECK(at_max->min_fl == 600);
  CHECK(at_max->max_fl == 600);
}

}  // namespace
