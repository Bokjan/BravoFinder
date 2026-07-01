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

}  // namespace bf
