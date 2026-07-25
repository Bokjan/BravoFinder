// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <memory>
#include <string>

#include "core/result.h"
#include "io/loaders/loader.h"

namespace bf {

// Construct the Loader for a source name (e.g. "xplane12"), or an Error with
// kInvalidArgument if the name is unknown. This is the single point that maps a
// `source_loader` string (from `bf build --loader` or a cache header) to a
// concrete implementation.
Result<std::unique_ptr<Loader>> MakeLoader(const std::string& name);

}  // namespace bf
