// SPDX-License-Identifier: MIT
#include "version_banner.h"

#include <format>

#include "core/version.h"

namespace bf::service {

std::string VersionBanner() {
  return std::format(
      "BravoFinder {}\n"
      "Copyright (c) Boyin Chen, and all contributors\n"
      "MIT-licensed, except the route engine (libs/engine/) which is under the GNU LGPL "
      "v3.0-or-later.\n"
      "See LICENSE.md, LICENSE.MIT, libs/engine/LICENSE and libs/engine/LICENSE.GPLv3.",
      bf::kBravoFinderVersion);
}

}  // namespace bf::service
