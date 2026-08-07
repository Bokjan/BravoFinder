// SPDX-License-Identifier: MIT
#pragma once

// version_banner.h — the shared --version text for bf / bf-http / bf-mcp.
//
// The three front-ends are combined works that statically link the LGPL route
// engine; LGPLv3 §4c requires the Library copyright notice to travel with the
// binary. Keeping the banner here (app-tier service layer) rather than in
// libs/engine/ keeps the engine embeddable under LGPL alone without carrying
// BravoFinder's combined-work notice.

#include <string>

namespace bf::service {

// Multi-line banner: "BravoFinder <version>" plus copyright / license pointers.
// Safe to pass to CLI11's set_version_flag.
std::string VersionBanner();

}  // namespace bf::service
