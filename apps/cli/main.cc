#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <CLI/CLI.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace {

// Print a route in human-readable text form.
void PrintText(const bf::Route& route) {
  std::cout << route.route_string << "\n\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "Total distance: " << route.total_distance_nm << " NM\n";

  // Surface the terminal procedures, if any, and the interchangeable choices
  // that share the same connection fix.
  if (!route.sid.empty()) {
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
  if (!route.star.empty()) {
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
    std::cout << leg.from << '\t' << leg.to << '\t' << leg.via << '\t' << leg.distance_nm << '\n';
  }
}

// Serialize one route into a rapidjson Writer. The Writer streams directly to
// the buffer (no intermediate DOM) and escapes strings correctly.
template <class Writer>
void WriteRouteJson(Writer& writer, const bf::Route& route) {
  auto string_array = [&](const std::vector<std::string>& items) {
    writer.StartArray();
    for (const std::string& s : items) {
      writer.String(s.c_str(), static_cast<rapidjson::SizeType>(s.size()));
    }
    writer.EndArray();
  };

  writer.StartObject();
  writer.Key("route");
  writer.String(route.route_string.c_str(),
                static_cast<rapidjson::SizeType>(route.route_string.size()));
  writer.Key("total_distance_nm");
  writer.Double(route.total_distance_nm);
  writer.Key("sid");
  writer.String(route.sid.c_str(), static_cast<rapidjson::SizeType>(route.sid.size()));
  writer.Key("dep_runway");
  writer.String(route.dep_runway.c_str(),
                static_cast<rapidjson::SizeType>(route.dep_runway.size()));
  writer.Key("sid_options");
  string_array(route.sid_options);
  writer.Key("star");
  writer.String(route.star.c_str(), static_cast<rapidjson::SizeType>(route.star.size()));
  writer.Key("arr_runway");
  writer.String(route.arr_runway.c_str(),
                static_cast<rapidjson::SizeType>(route.arr_runway.size()));
  writer.Key("star_options");
  string_array(route.star_options);
  writer.Key("legs");
  writer.StartArray();
  for (const bf::RouteLeg& leg : route.legs) {
    writer.StartObject();
    writer.Key("from");
    writer.String(leg.from.c_str(), static_cast<rapidjson::SizeType>(leg.from.size()));
    writer.Key("to");
    writer.String(leg.to.c_str(), static_cast<rapidjson::SizeType>(leg.to.size()));
    writer.Key("via");
    writer.String(leg.via.c_str(), static_cast<rapidjson::SizeType>(leg.via.size()));
    writer.Key("distance_nm");
    writer.Double(leg.distance_nm);
    writer.EndObject();
  }
  writer.EndArray();
  writer.EndObject();
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

}  // namespace

int main(int argc, char** argv) {
  CLI::App app{"BravoFinder - a flight route finder"};
  app.require_subcommand(1);

  CLI::App* route = app.add_subcommand("route", "Find a route between two points");
  std::string departure;
  std::string arrival;
  std::string data_dir = "navdata";
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

  CLI11_PARSE(app, argc, argv);

  if (*route) {
    bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(data_dir);
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

  return EXIT_SUCCESS;
}
