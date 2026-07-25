// SPDX-License-Identifier: LGPL-3.0-or-later
#include <optional>
#include <string>
#include <vector>

#include "core/util/string_util.h"
#include "io/graph_builder.h"
#include "io/nav_database.h"

namespace bf {

std::vector<MsaSector> NavDatabase::MsaForAirport(const std::string& icao) const {
  const std::string up = ToUpper(icao);
  std::vector<MsaSector> out;
  for (const MsaSector& s : msa_) {
    if (s.airport_icao == up) {
      out.push_back(s);
    }
  }
  return out;
}

std::vector<std::vector<WaypointInfo>> NavDatabase::LookupWaypoints(
    const std::vector<std::string>& idents) const {
  std::vector<std::vector<WaypointInfo>> out(idents.size());
  if (!builder_) {
    return out;
  }
  for (size_t i = 0; i < idents.size(); ++i) {
    const std::vector<int> vertices = builder_->VerticesByIdent(ToUpper(idents[i]));
    for (const int v : vertices) {
      // Airports share the ident namespace but are looked up via LookupAirports;
      // skip them here so a bare ICAO does not masquerade as a waypoint match.
      if (builder_->IsAirport(v)) {
        continue;
      }
      const Ident& id = builder_->IdentOf(v);
      out[i].push_back(WaypointInfo{id.ident, id.region, builder_->graph().CoordOf(v),
                                    builder_->KindOf(v), builder_->OnNetwork(v)});
    }
  }
  return out;
}

std::vector<std::optional<AirportInfo>> NavDatabase::LookupAirports(
    const std::vector<std::string>& icaos) const {
  std::vector<std::optional<AirportInfo>> out(icaos.size());
  if (!builder_) {
    return out;
  }
  for (size_t i = 0; i < icaos.size(); ++i) {
    const std::string up = ToUpper(icaos[i]);
    const int v = builder_->VertexByAirport(up);
    if (v < 0) {
      continue;
    }
    const Ident& id = builder_->IdentOf(v);
    out[i] = AirportInfo{id.ident, id.region, builder_->graph().CoordOf(v),
                         builder_->ElevationOf(v), ProceduresFor(up) != nullptr};
  }
  return out;
}

std::vector<std::optional<AirportProcedures>> NavDatabase::LookupProcedures(
    const std::vector<std::string>& icaos) const {
  std::vector<std::optional<AirportProcedures>> out(icaos.size());
  for (size_t i = 0; i < icaos.size(); ++i) {
    const std::string up = ToUpper(icaos[i]);
    const CifpData* cifp = ProceduresFor(up);
    if (cifp == nullptr) {
      continue;
    }
    AirportProcedures ap;
    ap.icao = up;
    ap.procedures.reserve(cifp->procedures.size());
    for (const Procedure& p : cifp->procedures) {
      ap.procedures.push_back(ProcedureSummary{p.type, p.name, p.transition_ident, p.runway});
    }
    out[i] = std::move(ap);
  }
  return out;
}

// Format a leg's altitude constraint as a compact display token. Empty when the
// leg carries no altitude restriction. "+"=at or above, "-"=at or below,
// "@"=at, "lo-hi"=between.
namespace {
std::string FormatAltToken(const AltitudeConstraint& a) {
  switch (a.kind) {
    case AltConstraintKind::kNone:
      return {};
    case AltConstraintKind::kAt:
      return "@" + std::to_string(a.alt1_ft);
    case AltConstraintKind::kAtOrAbove:
      return "+" + std::to_string(a.alt1_ft);
    case AltConstraintKind::kAtOrBelow:
      return "-" + std::to_string(a.alt1_ft);
    case AltConstraintKind::kBetween:
      return std::to_string(a.alt2_ft) + "-" + std::to_string(a.alt1_ft);
  }
  return {};
}
}  // namespace

std::optional<AirportProcedureDetail> NavDatabase::LookupProcedureDetail(
    const std::string& icao, const std::string& procedure_name) const {
  const std::string up_icao = ToUpper(icao);
  const std::string up_name = ToUpper(procedure_name);
  const CifpData* cifp = ProceduresFor(up_icao);
  if (cifp == nullptr) {
    return std::nullopt;
  }
  AirportProcedureDetail out;
  out.icao = up_icao;
  out.procedure = up_name;
  for (const Procedure& p : cifp->procedures) {
    if (ToUpper(p.name) != up_name) {
      continue;
    }
    ProcedureDetail d;
    d.type = p.type;
    d.name = p.name;
    d.transition = p.transition_ident;
    d.runway = p.runway;
    d.legs.reserve(p.legs.size());
    for (const ProcedureLeg& leg : p.legs) {
      ProcedureLegInfo info;
      info.fix = std::string(leg.fix.IdentView());
      info.path_term = PathTerminatorName(leg.path_term);
      info.course_deg = leg.course_deg;
      info.distance_nm = leg.distance_nm;
      info.alt = FormatAltToken(leg.alt);
      info.rnp_nm = leg.rnp_centinm / 100.0;
      info.turn_dir = leg.turn_dir;
      info.speed_limit_kt = leg.speed_limit_kt;
      d.legs.push_back(std::move(info));
    }
    out.transitions.push_back(std::move(d));
  }
  if (out.transitions.empty()) {
    return std::nullopt;  // airport has procedures, but none of that name
  }
  return out;
}

std::vector<std::optional<AirwayInfo>> NavDatabase::LookupAirways(
    const std::vector<std::string>& names) const {
  std::vector<std::optional<AirwayInfo>> out(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    auto it = airway_index_.find(ToUpper(names[i]));
    if (it != airway_index_.end()) {
      out[i] = it->second;
    }
  }
  return out;
}

std::vector<std::vector<NavaidDetailInfo>> NavDatabase::LookupNavaidDetails(
    const std::vector<std::string>& idents) const {
  std::vector<std::vector<NavaidDetailInfo>> out(idents.size());
  if (!detail_archive_.has_value()) {
    return out;
  }
  for (size_t i = 0; i < idents.size(); ++i) {
    out[i] = detail_archive_->FindNavaids(ToUpper(idents[i]));
  }
  return out;
}

std::vector<std::vector<HoldInfo>> NavDatabase::LookupHolds(
    const std::vector<std::string>& fix_idents) const {
  std::vector<std::vector<HoldInfo>> out(fix_idents.size());
  if (!detail_archive_.has_value()) {
    return out;
  }
  for (size_t i = 0; i < fix_idents.size(); ++i) {
    out[i] = detail_archive_->FindHolds(ToUpper(fix_idents[i]));
  }
  return out;
}

}  // namespace bf
