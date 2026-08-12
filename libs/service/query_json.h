// SPDX-License-Identifier: MIT
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/domain/msa.h"
#include "core/query/query_types.h"

namespace bf {

// Header-only JSON serialization for the batch lookup result types, over a
// RapidJSON-style Writer. Header-only so the service layer adds no RapidJSON
// dependency to the engine; callers link RapidJSON and pass a Writer. Mirrors
// service/route_json.h.

inline const char* ToString(WaypointKind k) {
  switch (k) {
    case WaypointKind::kFix:
      return "fix";
    case WaypointKind::kVor:
      return "vor";
    case WaypointKind::kNdb:
      return "ndb";
    case WaypointKind::kDme:
      return "dme";
    case WaypointKind::kOther:
      return "other";
  }
  return "other";
}

inline const char* ToString(ProcedureType t) {
  switch (t) {
    case ProcedureType::kSid:
      return "sid";
    case ProcedureType::kStar:
      return "star";
    case ProcedureType::kApproach:
      return "approach";
  }
  return "unknown";
}

namespace detail {

template <class Writer>
void WriteStr(Writer& w, std::string_view s) {
  w.String(s.data(), static_cast<unsigned>(s.size()));
}
template <class Writer>
void WriteStr(Writer& w, const std::string& s) {
  WriteStr(w, std::string_view(s));
}
template <class Writer>
void WriteKeyStr(Writer& w, const char* k, std::string_view s) {
  w.Key(k);
  WriteStr(w, s);
}
template <class Writer>
void WriteKeyStr(Writer& w, const char* k, const std::string& s) {
  WriteKeyStr(w, k, std::string_view(s));
}
template <class Writer>
void WriteCoord(Writer& w, const Coordinate& c) {
  w.Key("lat");
  w.Double(c.latitude);
  w.Key("lon");
  w.Double(c.longitude);
}

}  // namespace detail

template <class Writer>
void WriteWaypointJson(Writer& w, const WaypointInfo& wp) {
  w.StartObject();
  detail::WriteKeyStr(w, "ident", wp.ident);
  detail::WriteKeyStr(w, "arinc424_icao_code", wp.arinc424_icao_code);
  detail::WriteCoord(w, wp.coord);
  w.Key("kind");
  w.String(ToString(wp.kind));
  w.Key("on_network");
  w.Bool(wp.on_network);
  w.EndObject();
}

template <class Writer>
void WriteAirportJson(Writer& w, const AirportInfo& a) {
  w.StartObject();
  detail::WriteKeyStr(w, "icao", a.icao);
  detail::WriteKeyStr(w, "arinc424_icao_code", a.arinc424_icao_code);
  detail::WriteCoord(w, a.coord);
  w.Key("elevation_ft");
  w.Int(a.elevation_ft);
  w.Key("has_procedures");
  w.Bool(a.has_procedures);
  w.Key("procedures_corrupt");
  w.Bool(a.procedures_corrupt);
  w.EndObject();
}

template <class Writer>
void WriteProceduresJson(Writer& w, const AirportProcedures& ap) {
  w.StartObject();
  detail::WriteKeyStr(w, "icao", ap.icao);
  w.Key("procedures");
  w.StartArray();
  for (const ProcedureSummary& p : ap.procedures) {
    w.StartObject();
    w.Key("type");
    w.String(ToString(p.type));
    detail::WriteKeyStr(w, "name", p.name);
    detail::WriteKeyStr(w, "transition", p.transition);
    detail::WriteKeyStr(w, "runway", p.runway);
    w.EndObject();
  }
  w.EndArray();
  w.EndObject();
}

template <class Writer>
void WriteProcedureDetailJson(Writer& w, const AirportProcedureDetail& d) {
  w.StartObject();
  detail::WriteKeyStr(w, "icao", d.icao);
  detail::WriteKeyStr(w, "procedure", d.procedure);
  w.Key("transitions");
  w.StartArray();
  for (const ProcedureDetail& t : d.transitions) {
    w.StartObject();
    w.Key("type");
    w.String(ToString(t.type));
    detail::WriteKeyStr(w, "name", t.name);
    detail::WriteKeyStr(w, "transition", t.transition);
    detail::WriteKeyStr(w, "runway", t.runway);
    w.Key("legs");
    w.StartArray();
    for (const ProcedureLegInfo& leg : t.legs) {
      w.StartObject();
      detail::WriteKeyStr(w, "fix", leg.fix);
      detail::WriteKeyStr(w, "path_term", leg.path_term);
      w.Key("course_deg");
      w.Double(leg.course_deg);
      w.Key("distance_nm");
      w.Double(leg.distance_nm);
      // Omit absent optional fields (sentinel values) to keep output lean.
      if (!leg.alt.empty()) {
        detail::WriteKeyStr(w, "alt", leg.alt);
      }
      if (leg.rnp_nm > 0.0) {
        w.Key("rnp_nm");
        w.Double(leg.rnp_nm);
      }
      if (leg.turn_dir != '\0') {
        w.Key("turn_dir");
        w.String(leg.turn_dir == 'L' ? "L" : "R");
      }
      if (leg.speed_limit_kt > 0) {
        w.Key("speed_limit_kt");
        w.Int(leg.speed_limit_kt);
      }
      w.EndObject();
    }
    w.EndArray();
    w.EndObject();
  }
  w.EndArray();
  w.EndObject();
}

template <class Writer>
void WriteAirwayJson(Writer& w, const AirwayInfo& a) {
  w.StartObject();
  detail::WriteKeyStr(w, "name", a.name);
  w.Key("segments");
  w.StartArray();
  for (const AirwayLeg& s : a.segments) {
    w.StartObject();
    detail::WriteKeyStr(w, "from", s.from.View());
    detail::WriteKeyStr(w, "to", s.to.View());
    w.Key("distance_nm");
    w.Double(s.distance_nm);
    w.Key("high");
    w.Bool(s.high);
    w.Key("base_fl");
    w.Int(s.base_fl);
    w.Key("top_fl");
    w.Int(s.top_fl);
    w.EndObject();
  }
  w.EndArray();
  w.EndObject();
}

template <class Writer>
void WriteNavaidDetailJson(Writer& w, const NavaidDetailInfo& d) {
  w.StartObject();
  detail::WriteKeyStr(w, "ident", d.ident);
  detail::WriteKeyStr(w, "arinc424_icao_code", d.arinc424_icao_code);
  w.Key("kind");
  w.String(ToString(d.kind));
  w.Key("elev_ft");
  w.Int(d.elev_ft);
  w.Key("freq_raw");
  w.Int(d.freq_raw);
  w.Key("range_nm");
  w.Double(d.range_nm);
  w.Key("heading");
  w.Double(d.heading);
  w.EndObject();
}

template <class Writer>
void WriteHoldJson(Writer& w, const HoldInfo& h) {
  w.StartObject();
  detail::WriteKeyStr(w, "fix_ident", h.fix_ident);
  detail::WriteKeyStr(w, "fix_arinc424_icao_code", h.fix_arinc424_icao_code);
  detail::WriteKeyStr(w, "airport_icao", h.airport_icao);
  w.Key("inbound_course");
  w.Double(h.inbound_course);
  w.Key("leg_time_min");
  w.Double(h.leg_time_min);
  w.Key("leg_dist_nm");
  w.Double(h.leg_dist_nm);
  w.Key("turn_dir");
  w.String(h.turn_dir == 'L' ? "L" : "R");
  w.Key("min_alt_ft");
  w.Int(h.min_alt_ft);
  w.Key("max_alt_ft");
  w.Int(h.max_alt_ft);
  w.Key("speed_limit_kt");
  w.Int(h.speed_limit_kt);
  w.EndObject();
}

// One airport's MSA payload: `{icao, sectors:[{center_ident, center_arinc424_icao_code,
// arcs:[...]}]}`.
template <class Writer>
void WriteAirportMsaJson(Writer& w, std::string_view icao, const std::vector<MsaSector>& sectors) {
  w.StartObject();
  detail::WriteKeyStr(w, "icao", icao);
  w.Key("sectors");
  w.StartArray();
  for (const MsaSector& s : sectors) {
    w.StartObject();
    detail::WriteKeyStr(w, "center_ident", s.center.ident);
    detail::WriteKeyStr(w, "center_arinc424_icao_code", s.center.arinc424_icao_code);
    w.Key("arcs");
    w.StartArray();
    for (const MsaArc& a : s.arcs) {
      w.StartObject();
      w.Key("bearing_from");
      w.Int(a.bearing_from);
      w.Key("alt_100ft");
      w.Int(a.alt_100ft);
      w.Key("radius_nm");
      w.Int(a.radius_nm);
      w.EndObject();
    }
    w.EndArray();
    w.EndObject();
  }
  w.EndArray();
  w.EndObject();
}

}  // namespace bf
