// SPDX-License-Identifier: MIT
// render.cc — implementation of the query-layer renderers (see render.h).
//
// The JSON branch of each renderer is the serialization the transports ship
// (ported from the former handlers.cc, so MCP / HTTP output is byte-unchanged).
// The text branch is ported verbatim from the former CLI printers (cli_common.cc
// PrintText and cmd_query.cc RunQuery), so `bf ... --format text` output is
// byte-for-byte unchanged.

#include "render.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <format>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "core/domain/encoding_scale.h"
#include "core/routing/route.h"
#include "core/routing/route_metrics.h"
#include "query_json.h"
#include "route_json.h"

namespace bf::service {

namespace {

// A small RAII guard over a RapidJSON Writer writing to a StringBuffer, with the
// 6-dp max-decimal-places every JSON renderer here uses (point lat/lon must not
// truncate to ~1.1 km). Returns the buffer's string on destruction.
class JsonBuf {
 public:
  JsonBuf() { writer_.SetMaxDecimalPlaces(6); }
  rapidjson::Writer<rapidjson::StringBuffer>& writer() { return writer_; }
  std::string str() const { return buffer_.GetString(); }

 private:
  rapidjson::StringBuffer buffer_;
  rapidjson::Writer<rapidjson::StringBuffer> writer_{buffer_};
};

// ---- Route text (ported from cli_common.cc PrintText) -----------------------
void WriteRouteText(std::ostream& os, const bf::Route& route) {
  os << route.route_string << "\n\n";
  os << std::fixed << std::setprecision(1);
  os << "Total distance: " << route.total_distance_nm << " NM";
  // Break the total down by phase (dep procedure / enroute / arr procedure).
  os << "  (dep " << route.dep_distance_nm << " + enroute " << route.enroute_distance_nm
     << " + arr " << route.arr_distance_nm << ")\n";

  // Surface the terminal procedures, if any, and the interchangeable choices
  // that share the same connection fix. A radar-vectored departure/arrival has
  // no named procedure but is called out so it does not look like missing data.
  if (route.dep_connection == bf::ConnectionKind::kRadarVectors) {
    os << "SID: RADAR VECTORS\n";
  } else if (!route.sid.empty()) {
    os << "SID: " << route.sid;
    if (!route.dep_runway.empty()) {
      os << " (rwy " << route.dep_runway << ")";
    }
    if (route.sid_options.size() > 1) {
      os << " [options: ";
      for (size_t i = 0; i < route.sid_options.size(); ++i) {
        os << route.sid_options[i] << (i + 1 < route.sid_options.size() ? ", " : "");
      }
      os << "]";
    }
    os << "\n";
  }
  if (route.arr_connection == bf::ConnectionKind::kRadarVectors) {
    os << "STAR: RADAR VECTORS\n";
  } else if (route.arr_connection == bf::ConnectionKind::kTerminalTransition) {
    os << "[APCH PROC] " << route.approach;
    if (!route.approach_iaf.empty()) {
      os << " via " << route.approach_iaf;
    }
    if (!route.arr_runway.empty()) {
      os << " (rwy " << route.arr_runway << ")";
    }
    if (route.approach_options.size() > 1) {
      os << " [options: ";
      for (size_t i = 0; i < route.approach_options.size(); ++i) {
        os << route.approach_options[i] << (i + 1 < route.approach_options.size() ? ", " : "");
      }
      os << "]";
    }
    os << "\n";
  } else if (!route.star.empty()) {
    os << "STAR: " << route.star;
    if (!route.arr_runway.empty()) {
      os << " (rwy " << route.arr_runway << ")";
    }
    if (route.star_options.size() > 1) {
      os << " [options: ";
      for (size_t i = 0; i < route.star_options.size(); ++i) {
        os << route.star_options[i] << (i + 1 < route.star_options.size() ? ", " : "");
      }
      os << "]";
    }
    os << "\n";
  }

  if (!route.forced_points.empty()) {
    os << "Via: ";
    for (size_t i = 0; i < route.forced_points.size(); ++i) {
      os << route.forced_points[i] << (i + 1 < route.forced_points.size() ? ", " : "");
    }
    os << "\n";
  }

  os << "\nFrom\tTo\tVia\tDist(NM)\tCumul(NM)\tLat\tLon\n";
  const std::vector<double> cumulative = bf::CumulativeDistances(route.legs);
  for (size_t i = 0; i < route.legs.size(); ++i) {
    const bf::RouteLeg& leg = route.legs[i];
    // On a concurrency leg, note the other airways sharing it after the chosen
    // one, e.g. "Y592 (concurrent: A593, Y592)".
    std::string via = leg.via;
    if (!leg.concurrent_airways.empty()) {
      via += " (concurrent: ";
      for (size_t j = 0; j < leg.concurrent_airways.size(); ++j) {
        via += leg.concurrent_airways[j];
        via += (j + 1 < leg.concurrent_airways.size() ? ", " : ")");
      }
    }
    os << leg.from << '\t' << leg.to << '\t' << via << '\t' << leg.distance_nm << '\t'
       << cumulative[i] << '\t';
    // Coordinates of the leg's "to" point. points is parallel to legs with one
    // extra entry (N points, N-1 legs), so the destination of leg i is
    // points[i+1]; guard the size in case a route was built without points.
    if (i + 1 < route.points.size()) {
      os << std::setprecision(6) << route.points[i + 1].coord.latitude << '\t'
         << route.points[i + 1].coord.longitude << std::setprecision(1);
    } else {
      os << '\t';
    }
    os << '\n';
  }
}

// One airport's procedure summaries as text (shared by RenderProcedures and the
// summary branch of RenderProceduresMixed).
void WriteProceduresSummaryText(std::ostream& os, const bf::AirportProcedures& ap) {
  os << ap.icao << ": " << ap.procedures.size() << " procedures\n";
  for (const bf::ProcedureSummary& p : ap.procedures) {
    os << "  " << bf::ToString(p.type) << " " << p.name << "." << p.transition
       << (p.runway.empty() ? "" : "  rwy " + p.runway) << "\n";
  }
}

}  // namespace

std::string RenderRoutes(OutputFormat fmt, const std::vector<bf::Route>& routes,
                         uint32_t elapsed_ms) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const bf::Route& route : routes) {
      bf::WriteRouteJson(buf.writer(), route);
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < routes.size(); ++i) {
    if (routes.size() > 1) {
      os << "=== Route " << (i + 1) << " of " << routes.size() << " ===\n";
    }
    WriteRouteText(os, routes[i]);
    if (i + 1 < routes.size()) {
      os << "\n";
    }
  }
  os << "Query elapsed: " << elapsed_ms << " ms\n";
  return os.str();
}

std::string RenderRoute(OutputFormat fmt, const bf::Route& route, uint32_t elapsed_ms) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    bf::WriteRouteJson(buf.writer(), route);
    return buf.str();
  }
  std::ostringstream os;
  WriteRouteText(os, route);
  os << "Query elapsed: " << elapsed_ms << " ms\n";
  return os.str();
}

// ---- Batch lookups ----------------------------------------------------------

std::string RenderWaypoints(OutputFormat fmt, const std::vector<std::string>& ids,
                            const std::vector<std::vector<bf::WaypointInfo>>& results) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const auto& group : results) {
      buf.writer().StartArray();
      for (const bf::WaypointInfo& w : group) {
        bf::WriteWaypointJson(buf.writer(), w);
      }
      buf.writer().EndArray();
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (results[i].empty()) {
      os << ids[i] << ": not found\n";
      continue;
    }
    for (const bf::WaypointInfo& w : results[i]) {
      os << w.ident << " (" << w.arinc424_icao_code << ") " << bf::ToString(w.kind) << "  "
         << w.coord.latitude << ", " << w.coord.longitude << (w.on_network ? "  [on-network]" : "")
         << "\n";
    }
  }
  return os.str();
}

std::string RenderAirports(OutputFormat fmt, const std::vector<std::string>& ids,
                           const std::vector<std::optional<bf::AirportInfo>>& results) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const auto& opt : results) {
      if (opt) {
        bf::WriteAirportJson(buf.writer(), *opt);
      } else {
        buf.writer().Null();
      }
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (!results[i]) {
      os << ids[i] << ": not found\n";
      continue;
    }
    const bf::AirportInfo& a = *results[i];
    os << a.icao << " (" << a.arinc424_icao_code << ")  " << a.coord.latitude << ", "
       << a.coord.longitude << "  elev " << a.elevation_ft << " ft"
       << (a.has_procedures ? "  [has procedures]" : "") << "\n";
  }
  return os.str();
}

std::string RenderProcedures(OutputFormat fmt, const std::vector<std::string>& ids,
                             const std::vector<std::optional<bf::AirportProcedures>>& results) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const auto& opt : results) {
      if (opt) {
        bf::WriteProceduresJson(buf.writer(), *opt);
      } else {
        buf.writer().Null();
      }
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (!results[i]) {
      os << ids[i] << ": not found\n";
      continue;
    }
    WriteProceduresSummaryText(os, *results[i]);
  }
  return os.str();
}

std::string RenderAirways(OutputFormat fmt, const std::vector<std::string>& ids,
                          const std::vector<std::optional<bf::AirwayInfo>>& results) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const auto& opt : results) {
      if (opt) {
        bf::WriteAirwayJson(buf.writer(), *opt);
      } else {
        buf.writer().Null();
      }
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (!results[i]) {
      os << ids[i] << ": not found\n";
      continue;
    }
    const bf::AirwayInfo& a = *results[i];
    os << a.name << ": " << a.segments.size() << " segments\n";
    for (const bf::AirwayLeg& s : a.segments) {
      os << "  " << s.from << " -> " << s.to << "  " << s.distance_nm << " NM  "
         << (s.high ? "high" : "low") << "  FL" << s.base_fl << "-" << s.top_fl << "\n";
    }
  }
  return os.str();
}

std::string RenderNavaidDetails(OutputFormat fmt, const std::vector<std::string>& ids,
                                const std::vector<std::vector<bf::NavaidDetailInfo>>& results) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const auto& group : results) {
      buf.writer().StartArray();
      for (const bf::NavaidDetailInfo& d : group) {
        bf::WriteNavaidDetailJson(buf.writer(), d);
      }
      buf.writer().EndArray();
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (results[i].empty()) {
      os << ids[i] << ": not found\n";
      continue;
    }
    for (const bf::NavaidDetailInfo& d : results[i]) {
      // NDB frequencies are kHz; VOR/DME/ILS are MHz (raw = MHz * 100).
      os << d.ident << " (" << d.arinc424_icao_code << ") " << bf::ToString(d.kind) << "  elev "
         << d.elev_ft << " ft  freq ";
      if (d.kind == bf::WaypointKind::kNdb) {
        os << d.freq_raw << " kHz";
      } else {
        os << (d.freq_raw / kCentiScale) << " MHz";
      }
      os << "  range " << d.range_nm << " NM\n";
    }
  }
  return os.str();
}

std::string RenderHolds(OutputFormat fmt, const std::vector<std::string>& ids,
                        const std::vector<std::vector<bf::HoldInfo>>& results) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (const auto& group : results) {
      buf.writer().StartArray();
      for (const bf::HoldInfo& h : group) {
        bf::WriteHoldJson(buf.writer(), h);
      }
      buf.writer().EndArray();
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (results[i].empty()) {
      os << ids[i] << ": not found\n";
      continue;
    }
    for (const bf::HoldInfo& h : results[i]) {
      os << h.fix_ident << " (" << h.fix_arinc424_icao_code << ")  " << h.airport_icao
         << "  inbound " << h.inbound_course << "°  ";
      if (h.leg_dist_nm > 0) {
        os << "out " << h.leg_dist_nm << " NM  ";
      } else {
        os << "out " << h.leg_time_min << " min  ";
      }
      os << (h.turn_dir == 'L' ? 'L' : 'R') << "-turn  alt " << h.min_alt_ft << "-" << h.max_alt_ft
         << " ft";
      if (h.speed_limit_kt > 0) {
        os << "  " << h.speed_limit_kt << " kt";
      }
      os << "\n";
    }
  }
  return os.str();
}

// ---- Single procedure detail (ported from cmd_query.cc PrintProcedureDetail) --

std::string RenderProcedureDetail(OutputFormat fmt, const bf::AirportProcedureDetail& d) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    bf::WriteProcedureDetailJson(buf.writer(), d);
    return buf.str();
  }
  std::ostringstream os;
  os << d.icao << "/" << d.procedure << ": " << d.transitions.size() << " transitions\n";
  os << std::fixed;
  for (const bf::ProcedureDetail& t : d.transitions) {
    os << "  " << bf::ToString(t.type) << " " << t.name << "." << t.transition;
    if (!t.runway.empty()) {
      os << "  rwy " << t.runway;
    }
    os << "\n";
    for (const bf::ProcedureLegInfo& leg : t.legs) {
      os << "    " << leg.path_term;
      if (!leg.fix.empty()) {
        os << " " << leg.fix;
      }
      os << std::setprecision(1) << "  crs " << leg.course_deg << "  " << leg.distance_nm << " NM";
      if (!leg.alt.empty()) {
        os << "  alt " << leg.alt;
      }
      if (leg.rnp_nm > 0.0) {
        os << std::setprecision(2) << "  RNP " << leg.rnp_nm;
      }
      if (leg.turn_dir != '\0') {
        os << "  " << leg.turn_dir << "-turn";
      }
      if (leg.speed_limit_kt > 0) {
        os << "  " << leg.speed_limit_kt << " kt";
      }
      os << "\n";
    }
  }
  return os.str();
}

// ---- Mixed procedure selectors (ported from cmd_query.cc RunQuery) ----------

std::string RenderProceduresMixed(
    OutputFormat fmt, const std::vector<std::string>& labels,
    const std::vector<std::optional<bf::AirportProcedures>>& summaries,
    const std::vector<std::optional<bf::AirportProcedureDetail>>& details) {
  if (fmt == OutputFormat::kJson) {
    JsonBuf buf;
    buf.writer().StartArray();
    for (size_t i = 0; i < labels.size(); ++i) {
      if (summaries[i]) {
        bf::WriteProceduresJson(buf.writer(), *summaries[i]);
      } else if (details[i]) {
        bf::WriteProcedureDetailJson(buf.writer(), *details[i]);
      } else {
        buf.writer().Null();
      }
    }
    buf.writer().EndArray();
    return buf.str();
  }
  std::ostringstream os;
  for (size_t i = 0; i < labels.size(); ++i) {
    if (summaries[i]) {
      WriteProceduresSummaryText(os, *summaries[i]);
    } else if (details[i]) {
      os << RenderProcedureDetail(fmt, *details[i]);
    } else {
      os << labels[i] << ": not found\n";
    }
  }
  return os.str();
}

// ---- Error payload ----------------------------------------------------------

std::string JsonError(const std::string& message) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("error");
  writer.String(message.c_str(), static_cast<unsigned>(message.size()));
  writer.EndObject();
  return buffer.GetString();
}

std::string RenderError(OutputFormat fmt, const std::string& message) {
  if (fmt == OutputFormat::kJson) {
    return JsonError(message);
  }
  return std::format("{}{}\n", kTextErrorPrefix, message);
}

}  // namespace bf::service
