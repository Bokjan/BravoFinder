// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <string>
#include <vector>

#include "core/routing/route.h"

namespace bf {

// Split an airway designator field into its list of route names. A concurrency
// (two or more designators sharing one physical leg) is encoded "A593-Y592" in
// the source; a single airway is just "Y592"; "DCT" yields {"DCT"}. Real ATS
// route designators are letter+number with no internal hyphen, so '-' is an
// unambiguous separator. Source order is preserved.
std::vector<std::string> SplitDesignators(const std::string& designator_field);

// Fold an ordered list of route legs into an ICAO-style filed-flight-plan route
// string ("DEP SID FIX AWY FIX STAR ARR"), listing an airway only where it is
// joined or left and omitting the fixes passed through in between.
//
// Concurrent airways (two or more designators sharing one physical leg, encoded
// "A593-Y592" in the source) are handled by intersecting the designator sets of
// consecutive legs: a non-empty running intersection means the legs stay on a
// shared route and fold together; an empty one is a genuine airway change and
// breaks the group at that fix. The chosen designator is the first survivor of
// the intersection (deterministic, and valid on every leg in the group since
// the intersection is contained in each). "DCT" legs never fold, so every
// direct-leg fix is listed.
//
// Side effect on `legs`: each leg's `via` is rewritten to the single chosen
// designator, and `concurrent_airways` is set to all designators on that leg
// when it was a concurrency (left empty otherwise).
std::string BuildRouteString(const std::string& first_point, std::vector<RouteLeg>& legs);

}  // namespace bf
