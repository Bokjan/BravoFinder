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

}  // namespace bf
