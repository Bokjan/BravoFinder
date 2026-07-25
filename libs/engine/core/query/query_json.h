// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "core/query/query_types.h"

namespace bf {

// Header-only JSON serialization for the batch lookup result types, over a
// RapidJSON-style Writer. Header-only so core gains no dependency; callers link
// RapidJSON and pass a Writer. Mirrors core/routing/route_json.h.

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
  return "sid";
}

namespace detail {

template <class Writer>
void WriteStr(Writer& w, const std::string& s) {
  w.String(s.c_str(), static_cast<unsigned>(s.size()));
}
template <class Writer>
void WriteKeyStr(Writer& w, const char* k, const std::string& s) {
  w.Key(k);
  WriteStr(w, s);
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
  detail::WriteKeyStr(w, "region", wp.region);
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
  detail::WriteKeyStr(w, "region", a.region);
  detail::WriteCoord(w, a.coord);
  w.Key("elevation_ft");
  w.Int(a.elevation_ft);
  w.Key("has_procedures");
  w.Bool(a.has_procedures);
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
    detail::WriteKeyStr(w, "from", s.from);
    detail::WriteKeyStr(w, "to", s.to);
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
  detail::WriteKeyStr(w, "region", d.region);
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
  detail::WriteKeyStr(w, "fix_region", h.fix_region);
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

}  // namespace bf
