#include "core/routing/route_string.h"

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

namespace {

// Build a leg from just its endpoints and airway field; distance is irrelevant
// to route-string folding and left at zero.
bf::RouteLeg Leg(const std::string& from, const std::string& to, const std::string& via) {
  bf::RouteLeg leg;
  leg.from = from;
  leg.to = to;
  leg.via = via;
  return leg;
}

TEST_CASE("consecutive legs on one airway list it once and omit through-fixes", "[route_string]") {
  std::vector<bf::RouteLeg> legs{
      Leg("A", "B", "Y28"),
      Leg("B", "C", "Y28"),
      Leg("C", "D", "Y28"),
  };
  const std::string rs = bf::BuildRouteString("A", legs);
  CHECK(rs == "A Y28 D");
  // Every leg keeps the single designator; none is flagged as a concurrency.
  for (const bf::RouteLeg& leg : legs) {
    CHECK(leg.via == "Y28");
    CHECK(leg.concurrent_airways.empty());
  }
}

TEST_CASE("a concurrency flanked by a single airway converges to that airway", "[route_string]") {
  // The RJTT->ZSPD case: Y28, then V28-Y28 concurrency, then Y28 again. The
  // running intersection stays {Y28}, so no spurious hand-off fixes appear.
  std::vector<bf::RouteLeg> legs{
      Leg("IDNIL", "BIWWA", "Y28"),
      Leg("BIWWA", "MIDER", "V28-Y28"),
      Leg("MIDER", "SANDA", "Y28"),
  };
  const std::string rs = bf::BuildRouteString("IDNIL", legs);
  CHECK(rs == "IDNIL Y28 SANDA");
  CHECK(legs[0].via == "Y28");
  CHECK(legs[1].via == "Y28");
  CHECK(legs[2].via == "Y28");
  // The middle leg still records that it was physically a concurrency.
  CHECK(legs[1].concurrent_airways == std::vector<std::string>{"V28", "Y28"});
  CHECK(legs[0].concurrent_airways.empty());
  CHECK(legs[2].concurrent_airways.empty());
}

TEST_CASE("a concurrency that never narrows picks the lexicographically smallest",
          "[route_string]") {
  // DALTI->KABRA->PARLO both legs are G325-J133 and nothing outside narrows it;
  // the intersection stays {G325, J133}, so the deterministic pick is G325.
  std::vector<bf::RouteLeg> legs{
      Leg("DALTI", "KABRA", "G325-J133"),
      Leg("KABRA", "PARLO", "G325-J133"),
  };
  const std::string rs = bf::BuildRouteString("DALTI", legs);
  CHECK(rs == "DALTI G325 PARLO");
  CHECK(legs[0].via == "G325");
  CHECK(legs[1].via == "G325");
  CHECK(legs[0].concurrent_airways == std::vector<std::string>{"G325", "J133"});
  CHECK(legs[1].concurrent_airways == std::vector<std::string>{"G325", "J133"});
}

TEST_CASE("a genuine airway change breaks the group at the hand-off fix", "[route_string]") {
  std::vector<bf::RouteLeg> legs{
      Leg("A", "B", "V28"),
      Leg("B", "C", "V28-Y28"),
      Leg("C", "D", "Y28"),
  };
  // A-B is only V28; B-C is {V28,Y28}; C-D is only Y28. Intersection A..B..C is
  // {V28}, then C-D empties it -> hand off at C: A V28 C Y28 D.
  const std::string rs = bf::BuildRouteString("A", legs);
  CHECK(rs == "A V28 C Y28 D");
  CHECK(legs[0].via == "V28");
  CHECK(legs[1].via == "V28");
  CHECK(legs[2].via == "Y28");
  CHECK(legs[1].concurrent_airways == std::vector<std::string>{"V28", "Y28"});
}

TEST_CASE("multi-way concurrency intersects across all designators", "[route_string]") {
  std::vector<bf::RouteLeg> legs{
      Leg("A", "B", "J117-J167-J215"),
      Leg("B", "C", "A325-J117-J215"),
  };
  // Intersection is {J117, J215}; smallest is J117.
  const std::string rs = bf::BuildRouteString("A", legs);
  CHECK(rs == "A J117 C");
  CHECK(legs[0].via == "J117");
  CHECK(legs[1].via == "J117");
}

TEST_CASE("DCT legs never fold and list every fix", "[route_string]") {
  std::vector<bf::RouteLeg> legs{
      Leg("KJFK", "ROBER", "DCT"),
      Leg("ROBER", "MERIT", "Y28"),
      Leg("MERIT", "KBOS", "DCT"),
  };
  const std::string rs = bf::BuildRouteString("KJFK", legs);
  CHECK(rs == "KJFK DCT ROBER Y28 MERIT DCT KBOS");
  CHECK(legs[0].concurrent_airways.empty());
}

TEST_CASE("procedure legs stay as their own segments around the enroute airways",
          "[route_string]") {
  // Leading SID and trailing STAR are single tokens unequal to the airway, so
  // the intersection empties at each boundary and they stand alone.
  std::vector<bf::RouteLeg> legs{
      Leg("KJFK", "MERIT", "DEEZZ5"),
      Leg("MERIT", "IGN", "Y28"),
      Leg("IGN", "KLAX", "CAMRN5"),
  };
  const std::string rs = bf::BuildRouteString("KJFK", legs);
  CHECK(rs == "KJFK DEEZZ5 MERIT Y28 IGN CAMRN5 KLAX");
}

TEST_CASE("empty legs yield just the first point", "[route_string]") {
  std::vector<bf::RouteLeg> legs;
  CHECK(bf::BuildRouteString("KJFK", legs) == "KJFK");
}

TEST_CASE("SplitDesignators drops empty segments", "[route_string]") {
  // A leading/trailing/double hyphen or an empty input must not produce empty
  // designators: an empty string would be written into the route string and
  // mistaken for a real airway name.
  CHECK(bf::SplitDesignators("").empty());
  CHECK(bf::SplitDesignators("A593-").size() == 1);
  CHECK(bf::SplitDesignators("A593-")[0] == "A593");
  CHECK(bf::SplitDesignators("-A593").size() == 1);
  CHECK(bf::SplitDesignators("-A593")[0] == "A593");
  CHECK(bf::SplitDesignators("A593--Y592").size() == 2);
  CHECK(bf::SplitDesignators("A593--Y592")[0] == "A593");
  CHECK(bf::SplitDesignators("A593--Y592")[1] == "Y592");
  // A normal concurrency and a single designator are unaffected.
  CHECK(bf::SplitDesignators("A593-Y592").size() == 2);
  CHECK(bf::SplitDesignators("DCT").size() == 1);
}

}  // namespace
