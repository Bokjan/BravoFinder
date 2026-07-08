#include "cli_common.h"

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <charconv>
#include <iomanip>
#include <iostream>

#include "core/routing/route_json.h"

namespace bf::cli {

Result<NavDatabase> OpenForRead(const std::string& db_path, const std::string& data_dir,
                                const std::string& cifp_load) {
  if (db_path.empty()) {
    return NavDatabase::Open(data_dir);
  }
  return NavDatabase::OpenCached(db_path,
                                 cifp_load == "eager" ? CifpLoad::kEager : CifpLoad::kOnDemand);
}

void PrintText(const Route& route) {
  std::cout << route.route_string << "\n\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "Total distance: " << route.total_distance_nm << " NM\n";

  // Surface the terminal procedures, if any, and the interchangeable choices
  // that share the same connection fix. A radar-vectored departure/arrival has
  // no named procedure but is called out so it does not look like missing data.
  if (route.dep_connection == ConnectionKind::kRadarVectors) {
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
  if (route.arr_connection == ConnectionKind::kRadarVectors) {
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

  if (!route.forced_points.empty()) {
    std::cout << "Via: ";
    for (size_t i = 0; i < route.forced_points.size(); ++i) {
      std::cout << route.forced_points[i] << (i + 1 < route.forced_points.size() ? ", " : "");
    }
    std::cout << "\n";
  }

  std::cout << "\nFrom\tTo\tVia\tDist(NM)\n";
  for (const RouteLeg& leg : route.legs) {
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

void PrintRoutesJson(const std::vector<Route>& routes) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.SetMaxDecimalPlaces(2);
  writer.StartArray();
  for (const Route& route : routes) {
    WriteRouteJson(writer, route);
  }
  writer.EndArray();
  std::cout << buffer.GetString() << "\n";
}

std::optional<FlRange> ParseAltSpec(const std::string& spec) {
  const size_t dash = spec.find('-');
  auto to_int = [](const std::string& s, int& out) -> bool {
    if (s.empty()) {
      return false;
    }
    // std::from_chars is the exception-free counterpart of stoi: it fails via
    // an error code (no try/catch), and ptr == end verifies the whole field
    // was numeric. A leading '+' or whitespace is rejected, which is fine for
    // a flight-level spec.
    const char* begin = s.data();
    const char* end = begin + s.size();
    auto [ptr, ec] = std::from_chars(begin, end, out);
    return ec == std::errc{} && ptr == end && out >= 0;
  };
  if (dash == std::string::npos) {
    int fl = 0;
    if (!to_int(spec, fl)) {
      return std::nullopt;
    }
    return FlRange{fl, fl};
  }
  int lo = 0;
  int hi = 0;
  if (!to_int(spec.substr(0, dash), lo) || !to_int(spec.substr(dash + 1), hi)) {
    return std::nullopt;
  }
  if (lo > hi) {
    return std::nullopt;
  }
  return FlRange{lo, hi};
}

}  // namespace bf::cli
