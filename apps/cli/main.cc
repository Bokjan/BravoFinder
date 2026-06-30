#include <CLI/CLI.hpp>
#include <cstdlib>
#include <iostream>
#include <string>

#include "core/result.h"
#include "core/routing/route.h"
#include "core/routing/route_request.h"
#include "io/nav_database.h"

namespace {

// Print a route in human-readable text form.
void PrintText(const bf::Route& route) {
  std::cout << route.route_string << "\n\n";
  std::cout << "Total distance: " << route.total_distance_nm << " NM\n\n";
  std::cout << "From\tTo\tVia\tDist(NM)\n";
  for (const bf::RouteLeg& leg : route.legs) {
    std::cout << leg.from << '\t' << leg.to << '\t' << leg.via << '\t' << leg.distance_nm << '\n';
  }
}

// Print a route as a JSON object (hand-rolled to avoid a JSON dependency).
void PrintJson(const bf::Route& route) {
  std::cout << "{\n";
  std::cout << "  \"route\": \"" << route.route_string << "\",\n";
  std::cout << "  \"total_distance_nm\": " << route.total_distance_nm << ",\n";
  std::cout << "  \"legs\": [\n";
  for (size_t i = 0; i < route.legs.size(); ++i) {
    const bf::RouteLeg& leg = route.legs[i];
    std::cout << "    {\"from\": \"" << leg.from << "\", \"to\": \"" << leg.to << "\", \"via\": \""
              << leg.via << "\", \"distance_nm\": " << leg.distance_nm << "}"
              << (i + 1 < route.legs.size() ? "," : "") << "\n";
  }
  std::cout << "  ]\n";
  std::cout << "}\n";
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
  route->add_option("departure", departure, "Departure ICAO or waypoint ident")->required();
  route->add_option("arrival", arrival, "Arrival ICAO or waypoint ident")->required();
  route->add_option("--data", data_dir, "Directory of X-Plane navigation data")
      ->capture_default_str();
  route->add_option("--format", format, "Output format: text or json")
      ->capture_default_str()
      ->check(CLI::IsMember({"text", "json"}));

  CLI11_PARSE(app, argc, argv);

  if (*route) {
    bf::Result<bf::NavDatabase> db = bf::NavDatabase::Open(data_dir);
    if (!db) {
      std::cerr << "error: " << db.error().message << "\n";
      return EXIT_FAILURE;
    }
    bf::RouteRequest request{departure, arrival};
    bf::Result<bf::Route> result = db.value().FindRoute(request);
    if (!result) {
      std::cerr << "error: " << result.error().message << "\n";
      return EXIT_FAILURE;
    }
    if (format == "json") {
      PrintJson(result.value());
    } else {
      PrintText(result.value());
    }
  }

  return EXIT_SUCCESS;
}
