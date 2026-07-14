#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "cli_common.h"
#include "commands.h"
#include "core/query/query_json.h"
#include "io/nav_database.h"

namespace bf::cli {

namespace {

// Print one named procedure's transitions and their legs (the ICAO/NAME query
// form). One line per leg, omitting fields the leg does not carry.
void PrintProcedureDetail(const AirportProcedureDetail& d) {
  std::cout << d.icao << "/" << d.procedure << ": " << d.transitions.size() << " transitions\n";
  std::cout << std::fixed;
  for (const ProcedureDetail& t : d.transitions) {
    std::cout << "  " << ToString(t.type) << " " << t.name << "." << t.transition;
    if (!t.runway.empty()) {
      std::cout << "  rwy " << t.runway;
    }
    std::cout << "\n";
    for (const ProcedureLegInfo& leg : t.legs) {
      std::cout << "    " << leg.path_term;
      if (!leg.fix.empty()) {
        std::cout << " " << leg.fix;
      }
      std::cout << std::setprecision(1) << "  crs " << leg.course_deg << "  " << leg.distance_nm
                << " NM";
      if (!leg.alt.empty()) {
        std::cout << "  alt " << leg.alt;
      }
      if (leg.rnp_nm > 0.0) {
        std::cout << std::setprecision(2) << "  RNP " << leg.rnp_nm;
      }
      if (leg.turn_dir != '\0') {
        std::cout << "  " << leg.turn_dir << "-turn";
      }
      if (leg.speed_limit_kt > 0) {
        std::cout << "  " << leg.speed_limit_kt << " kt";
      }
      std::cout << "\n";
    }
  }
}

// Run one batch query and print its results (text or JSON). The lookup returns a
// vector parallel to `ids`; a missing entry is reported as not found. Returns
// the number of ids that were not found.
int RunQuery(const NavDatabase& db, const std::string& kind, const std::vector<std::string>& ids,
             const std::string& format) {
  const bool json = format == "json";
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(6);
  if (json) {
    writer.StartArray();
  }
  int not_found = 0;

  // Emit a JSON null (batch mode keeps output parallel to the input) or a text
  // "not found" line for a missing id.
  auto miss = [&](const std::string& id) {
    ++not_found;
    if (json) {
      writer.Null();
    } else {
      std::cout << id << ": not found\n";
    }
  };

  if (kind == "waypoint") {
    auto results = db.LookupWaypoints(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (results[i].empty()) {
        miss(ids[i]);
        continue;
      }
      for (const WaypointInfo& w : results[i]) {
        if (json) {
          WriteWaypointJson(writer, w);
        } else {
          std::cout << w.ident << " (" << w.region << ") " << ToString(w.kind) << "  "
                    << w.coord.latitude << ", " << w.coord.longitude
                    << (w.on_network ? "  [on-network]" : "") << "\n";
        }
      }
    }
  } else if (kind == "airport") {
    auto results = db.LookupAirports(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!results[i]) {
        miss(ids[i]);
        continue;
      }
      const AirportInfo& a = *results[i];
      if (json) {
        WriteAirportJson(writer, a);
      } else {
        std::cout << a.icao << " (" << a.region << ")  " << a.coord.latitude << ", "
                  << a.coord.longitude << "  elev " << a.elevation_ft << " ft"
                  << (a.has_procedures ? "  [has procedures]" : "") << "\n";
      }
    }
  } else if (kind == "procedure") {
    // Two forms: a bare ICAO lists procedure summaries; an "ICAO/NAME" selector
    // (e.g. KJFK/DEEZZ5) prints that named procedure's per-leg detail.
    for (size_t i = 0; i < ids.size(); ++i) {
      const std::string& id = ids[i];
      const size_t slash = id.find('/');
      if (slash != std::string::npos) {
        const AirportProcedureDetail* detail_ptr = nullptr;
        std::optional<AirportProcedureDetail> detail =
            db.LookupProcedureDetail(id.substr(0, slash), id.substr(slash + 1));
        detail_ptr = detail ? &*detail : nullptr;
        if (detail_ptr == nullptr) {
          miss(id);
          continue;
        }
        if (json) {
          WriteProcedureDetailJson(writer, *detail_ptr);
        } else {
          PrintProcedureDetail(*detail_ptr);
        }
        continue;
      }
      std::optional<AirportProcedures> ap = std::move(db.LookupProcedures({id})[0]);
      if (!ap) {
        miss(id);
        continue;
      }
      if (json) {
        WriteProceduresJson(writer, *ap);
      } else {
        std::cout << ap->icao << ": " << ap->procedures.size() << " procedures\n";
        for (const ProcedureSummary& p : ap->procedures) {
          std::cout << "  " << ToString(p.type) << " " << p.name << "." << p.transition
                    << (p.runway.empty() ? "" : "  rwy " + p.runway) << "\n";
        }
      }
    }
  } else if (kind == "navaid_detail") {
    auto results = db.LookupNavaidDetails(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (results[i].empty()) {
        miss(ids[i]);
        continue;
      }
      for (const NavaidDetailInfo& d : results[i]) {
        if (json) {
          WriteNavaidDetailJson(writer, d);
        } else {
          // NDB frequencies are kHz; VOR/DME/ILS are MHz (raw = MHz * 100).
          std::cout << d.ident << " (" << d.region << ") " << ToString(d.kind) << "  elev "
                    << d.elev_ft << " ft  freq ";
          if (d.kind == WaypointKind::kNdb) {
            std::cout << d.freq_raw << " kHz";
          } else {
            std::cout << (d.freq_raw / 100.0) << " MHz";
          }
          std::cout << "  range " << d.range_nm << " NM\n";
        }
      }
    }
  } else if (kind == "hold") {
    auto results = db.LookupHolds(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (results[i].empty()) {
        miss(ids[i]);
        continue;
      }
      for (const HoldInfo& h : results[i]) {
        if (json) {
          WriteHoldJson(writer, h);
        } else {
          std::cout << h.fix_ident << " (" << h.fix_region << ")  " << h.airport_icao
                    << "  inbound " << h.inbound_course << "°  ";
          if (h.leg_dist_nm > 0) {
            std::cout << "out " << h.leg_dist_nm << " NM  ";
          } else {
            std::cout << "out " << h.leg_time_min << " min  ";
          }
          std::cout << (h.turn_dir == 'L' ? 'L' : 'R') << "-turn  alt " << h.min_alt_ft << "-"
                    << h.max_alt_ft << " ft";
          if (h.speed_limit_kt > 0) {
            std::cout << "  " << h.speed_limit_kt << " kt";
          }
          std::cout << "\n";
        }
      }
    }
  } else {  // airway
    auto results = db.LookupAirways(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!results[i]) {
        miss(ids[i]);
        continue;
      }
      const AirwayInfo& a = *results[i];
      if (json) {
        WriteAirwayJson(writer, a);
      } else {
        std::cout << a.name << ": " << a.segments.size() << " segments\n";
        for (const AirwayLeg& s : a.segments) {
          std::cout << "  " << s.from << " -> " << s.to << "  " << s.distance_nm << " NM  "
                    << (s.high ? "high" : "low") << "  FL" << s.base_fl << "-" << s.top_fl << "\n";
        }
      }
    }
  }

  if (json) {
    writer.EndArray();
    std::cout << buffer.GetString() << "\n";
  }
  return not_found;
}

}  // namespace

void RegisterQuery(CLI::App& app, int& exit_code) {
  struct Args {
    std::string kind;
    std::vector<std::string> ids;
    std::string data_dir = "navdata";
    std::string db_path;
    std::string format = "text";
  };
  auto a = std::make_shared<Args>();

  CLI::App* query = app.add_subcommand(
      "query", "Look up navigation data (waypoints, airports, procedures, airways)");
  query
      ->add_option("kind", a->kind,
                   "What to look up: waypoint, airport, procedure, airway, navaid_detail, or hold")
      ->required()
      ->check(
          CLI::IsMember({"waypoint", "airport", "procedure", "airway", "navaid_detail", "hold"}));
  query
      ->add_option("id", a->ids,
                   "One or more idents / ICAO codes / airway names. For 'procedure', an "
                   "ICAO/NAME selector (e.g. KJFK/DEEZZ5) prints that procedure's per-leg detail")
      ->required();
  query->add_option("--data", a->data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  query->add_option("--db", a->db_path, "Prebuilt .bfdb cache to load (skips parsing)");
  query->add_option("--format", a->format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  query->callback([a, &exit_code]() {
    Result<NavDatabase> db = OpenForRead(a->db_path, a->data_dir);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      exit_code = EXIT_FAILURE;
      return;
    }
    const int not_found = RunQuery(db.value(), a->kind, a->ids, a->format);
    // Exit non-zero only if every requested id was missing, so scripts can
    // detect a wholly failed lookup; a partial hit still succeeds.
    if (not_found == static_cast<int>(a->ids.size())) {
      exit_code = EXIT_FAILURE;
    }
  });
}

}  // namespace bf::cli
