// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

namespace bf {

// Scale for packing a real quantity into centi-units: RNP in hundredths of a
// nautical mile (ProcedureLeg::rnp_centinm) and VHF navaid frequency as MHz*100
// (NavaidDetail::freq_raw). Lives in core/domain so query/render paths can
// decode without including loader-private headers.

inline constexpr double kCentiScale = 100.0;

}  // namespace bf
