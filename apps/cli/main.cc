#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <CLI/CLI.hpp>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "core/result.h"
#include "core/query/query_json.h"
#include "core/routing/route.h"
#include "core/routing/route_json.h"
#include "core/routing/route_request.h"
#include "core/version.h"
#include "io/nav_database.h"

namespace {

// Print a route in human-readable text form.
void PrintText(const bf::Route& route) {
  std::cout << route.route_string << "\n\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "Total distance: " << route.total_distance_nm << " NM\n";

  // Surface the terminal procedures, if any, and the interchangeable choices
  // that share the same connection fix. A radar-vectored departure/arrival has
  // no named procedure but is called out so it does not look like missing data.
  if (route.dep_connection == bf::ConnectionKind::kRadarVectors) {
    std::cout << "SID: RADAR VECTORS\n";
  } else if (!route.sid.empty()) {
    std::cout << "SID: " << route.sid;
    if (!route.dep_runway.empty()) {
      std::cout << " (rwy " << route.dep_runway << ")";
    }
    if (route.sid_options.size() > 1) {
      std::cout << " [options: ";
      for (size_t i = 0; i < route.sid_options.size(); ++i) {
        std::cout << route.sid_options[i] << (i + 1 < route.sid_options.size() ? ", " : "");
      }
      std::cout << "]";
    }
    std::cout << "\n";
  }
  if (route.arr_connection == bf::ConnectionKind::kRadarVectors) {
    std::cout << "STAR: RADAR VECTORS\n";
  } else if (!route.star.empty()) {
    std::cout << "STAR: " << route.star;
    if (!route.arr_runway.empty()) {
      std::cout << " (rwy " << route.arr_runway << ")";
    }
    if (route.star_options.size() > 1) {
      std::cout << " [options: ";
      for (size_t i = 0; i < route.star_options.size(); ++i) {
        std::cout << route.star_options[i] << (i + 1 < route.star_options.size() ? ", " : "");
      }
      std::cout << "]";
    }
    std::cout << "\n";
  }

  std::cout << "\nFrom\tTo\tVia\tDist(NM)\n";
  for (const bf::RouteLeg& leg : route.legs) {
    // On a concurrency leg, note the other airways sharing it after the chosen
    // one, e.g. "Y592 (concurrent: A593, Y592)".
    std::string via = leg.via;
    if (!leg.concurrent_airways.empty()) {
      via += " (concurrent: ";
      for (size_t i = 0; i < leg.concurrent_airways.size(); ++i) {
        via += leg.concurrent_airways[i];
        via += (i + 1 < leg.concurrent_airways.size() ? ", " : ")");
      }
    }
    std::cout << leg.from << '\t' << leg.to << '\t' << via << '\t' << leg.distance_nm << '\n';
  }
}

// Print the candidate routes as a JSON array.
void PrintRoutesJson(const std::vector<bf::Route>& routes) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(2);
  writer.StartArray();
  for (const bf::Route& route : routes) {
    WriteRouteJson(writer, route);
  }
  writer.EndArray();
  std::cout << buffer.GetString() << "\n";
}

// Run one batch query and print its results (text or JSON). The lookup returns a
// vector<optional<Info>> parallel to `ids`; a nullopt entry is reported as not
// found. Returns the number of ids that were not found.
int RunQuery(const bf::NavDatabase& db, const std::string& kind,
             const std::vector<std::string>& ids, const std::string& format) {
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
      if (!results[i]) {
        miss(ids[i]);
        continue;
      }
      const bf::WaypointInfo& w = *results[i];
      if (json) {
        bf::WriteWaypointJson(writer, w);
      } else {
        std::cout << w.ident << " (" << w.region << ") " << bf::ToString(w.kind) << "  "
                  << w.coord.latitude << ", " << w.coord.longitude
                  << (w.on_network ? "  [on-network]" : "") << "\n";
      }
    }
  } else if (kind == "airport") {
    auto results = db.LookupAirports(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!results[i]) {
        miss(ids[i]);
        continue;
      }
      const bf::AirportInfo& a = *results[i];
      if (json) {
        bf::WriteAirportJson(writer, a);
      } else {
        std::cout << a.icao << " (" << a.region << ")  " << a.coord.latitude << ", "
                  << a.coord.longitude << "  elev " << a.elevation_ft << " ft"
                  << (a.has_procedures ? "  [has procedures]" : "") << "\n";
      }
    }
  } else if (kind == "procedure") {
    auto results = db.LookupProcedures(ids);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!results[i]) {
        miss(ids[i]);
        continue;
      }
      const bf::AirportProcedures& ap = *results[i];
      if (json) {
        bf::WriteProceduresJson(writer, ap);
      } else {
        std::cout << ap.icao << ": " << ap.procedures.size() << " procedures\n";
        for (const bf::ProcedureSummary& p : ap.procedures) {
          std::cout << "  " << bf::ToString(p.type) << " " << p.name << "." << p.transition
                    << (p.runway.empty() ? "" : "  rwy " + p.runway) << "\n";
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
      const bf::AirwayInfo& a = *results[i];
      if (json) {
        bf::WriteAirwayJson(writer, a);
      } else {
        std::cout << a.name << ": " << a.segments.size() << " segments\n";
        for (const bf::AirwayLeg& s : a.segments) {
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

int main(int argc, char** argv) {
  CLI::App app{"BravoFinder - a flight route finder"};
  app.require_subcommand(1);
  app.set_version_flag("--version", bf::kBravoFinderVersion);

  // --- build: parse X-Plane data, build the graph, write a .bfdb cache. ---
  CLI::App* build = app.add_subcommand("build", "Build a .bfdb cache from X-Plane data");
  std::string build_data_dir;
  std::string build_output;
  std::string build_loader = "xplane";
  bool build_without_cifp = false;
  build->add_option("data_dir", build_data_dir, "Directory of X-Plane navigation data")->required();
  build->add_option("-o,--output", build_output,
                    "Output .bfdb path (default: <data_dir>/nav.bfdb)");
  build->add_option("--loader", build_loader, "Data source loader")
      ->capture_default_str()
      ->check(CLI::IsMember({"xplane"}));
  build->add_flag("--without-cifp", build_without_cifp,
                  "Skip building the CIFP procedure cache (<stem>_cifp.bfdb)");

  CLI::App* route = app.add_subcommand("route", "Find a route between two points");
  std::string departure;
  std::string arrival;
  std::string data_dir = "navdata";
  std::string db_path;
  std::string cifp_db_path;
  std::string cifp_load = "on-demand";
  std::string format = "text";
  std::string level = "none";
  std::optional<int> cruise_fl;
  std::string rwy_dep;
  std::string rwy_arr;
  int k = 1;
  route->add_option("departure", departure, "Departure ICAO or waypoint ident")->required();
  route->add_option("arrival", arrival, "Arrival ICAO or waypoint ident")->required();
  route->add_option("--data", data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  route->add_option("--db", db_path,
                    "Prebuilt .bfdb cache to load (skips parsing; --data still "
                    "locates CIFP files for procedures)");
  route->add_option("--cifp-db", cifp_db_path,
                    "CIFP procedure cache to load (default: <db-stem>_cifp.bfdb "
                    "next to --db, if present)");
  route->add_option("--cifp-load", cifp_load, "CIFP cache load mode: on-demand (default) or eager")
      ->capture_default_str()
      ->check(CLI::IsMember({"on-demand", "eager"}));
  route->add_option("--format", format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));
  route->add_option("--alt", cruise_fl,
                    "Cruise flight level, e.g. 350 (enables altitude/MORA filters)");
  route->add_option("--level", level, "Airway level preference: none, low, high")
      ->capture_default_str()
      ->check(CLI::IsMember({"none", "low", "high"}));
  route->add_option("-k", k, "Number of candidate routes to return")
      ->capture_default_str()
      ->check(CLI::PositiveNumber);
  route->add_option("--rwy-dep", rwy_dep,
                    "Departure runway to restrict the SID, e.g. RW31L (default: any)");
  route->add_option("--rwy-arr", rwy_arr,
                    "Arrival runway to restrict the STAR, e.g. RW25L (default: any)");
  std::string sid;
  std::string star;
  route->add_option("--sid", sid,
                    "Select a specific SID by name, e.g. DEEZZ5 or DEEZZ5.TOWIN (default: auto)");
  route->add_option("--star", star,
                    "Select a specific STAR by name, e.g. LENDY6 or LENDY6.HAAYS (default: auto)");

  // --- query: look up waypoints / airports / procedures / airways. ---
  CLI::App* query =
      app.add_subcommand("query", "Look up navigation data (waypoints, airports, procedures, airways)");
  std::string query_kind;
  std::vector<std::string> query_ids;
  std::string query_data_dir = "navdata";
  std::string query_db_path;
  std::string query_cifp_db_path;
  std::string query_format = "text";
  query->add_option("kind", query_kind, "What to look up: waypoint, airport, procedure, or airway")
      ->required()
      ->check(CLI::IsMember({"waypoint", "airport", "procedure", "airway"}));
  query->add_option("id", query_ids, "One or more idents / ICAO codes / airway names")->required();
  query->add_option("--data", query_data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  query->add_option("--db", query_db_path, "Prebuilt .bfdb cache to load (skips parsing)");
  query->add_option("--cifp-db", query_cifp_db_path,
                    "CIFP procedure cache to load (default: <db-stem>_cifp.bfdb next to --db)");
  query->add_option("--format", query_format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  CLI11_PARSE(app, argc, argv);

  if (*build) {
    const std::string out = build_output.empty() ? build_data_dir + "/nav.bfdb" : build_output;
    bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(build_data_dir);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      return EXIT_FAILURE;
    }
    bf::Result<void> written = db.value().WriteCache(out);
    if (!written) {
      std::cerr << "error: " << written.error().message << "\n";
      return EXIT_FAILURE;
    }
    std::cout << "wrote " << out << "\n";

    // Also build the CIFP procedure cache unless opted out, so deployment needs
    // only the cache files. Its path mirrors the graph cache: <stem>_cifp.bfdb.
    if (!build_without_cifp) {
      const std::filesystem::path p(out);
      const std::string cifp_out = (p.parent_path() / (p.stem().string() + "_cifp.bfdb")).string();
      bf::Result<uint32_t> n = db.value().WriteCifpCache(cifp_out, build_loader);
      if (!n) {
        std::cerr << "error: " << n.error().message << "\n";
        return EXIT_FAILURE;
      }
      std::cout << "wrote " << cifp_out << " (" << n.value() << " airports)\n";
    }
    return EXIT_SUCCESS;
  }

  if (*route) {
    // With --db, load the prebuilt cache (milliseconds); otherwise parse and
    // build from the data directory (M1 path). --data still locates CIFP files
    // for on-demand procedure parsing in both cases; --cifp-db (or a sibling
    // <stem>_cifp.bfdb) supplies procedures from a cache instead.
    bf::Result<bf::NavDatabase> db =
        db_path.empty()
            ? bf::NavDatabase::Open(data_dir)
            : bf::NavDatabase::OpenCached(
                  db_path, data_dir, cifp_db_path,
                  cifp_load == "eager" ? bf::CifpLoad::kEager : bf::CifpLoad::kOnDemand);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      return EXIT_FAILURE;
    }

    bf::RouteRequest request;
    request.departure = departure;
    request.arrival = arrival;
    request.cruise_fl = cruise_fl;
    request.k = k;
    request.departure_runway = rwy_dep;
    request.arrival_runway = rwy_arr;
    request.departure_sid = sid;
    request.arrival_star = star;
    if (level == "low") {
      request.level = bf::LevelPreference::kLow;
    } else if (level == "high") {
      request.level = bf::LevelPreference::kHigh;
    }

    bf::Result<std::vector<bf::Route>> result = db.value().FindRoutes(request);
    if (!result) {
      std::cerr << "error: " << result.error().message << "\n";
      return EXIT_FAILURE;
    }

    const std::vector<bf::Route>& routes = result.value();
    if (format == "json") {
      PrintRoutesJson(routes);
    } else {
      for (size_t i = 0; i < routes.size(); ++i) {
        if (routes.size() > 1) {
          std::cout << "=== Route " << (i + 1) << " of " << routes.size() << " ===\n";
        }
        PrintText(routes[i]);
        if (i + 1 < routes.size()) {
          std::cout << "\n";
        }
      }
    }
  }

  if (*query) {
    bf::Result<bf::NavDatabase> db =
        query_db_path.empty()
            ? bf::NavDatabase::Open(query_data_dir)
            : bf::NavDatabase::OpenCached(query_db_path, query_data_dir, query_cifp_db_path);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      return EXIT_FAILURE;
    }
    const int not_found = RunQuery(db.value(), query_kind, query_ids, query_format);
    // Exit non-zero if every requested id was missing, so scripts can detect a
    // wholly failed lookup; a partial hit still succeeds.
    if (not_found == static_cast<int>(query_ids.size())) {
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
