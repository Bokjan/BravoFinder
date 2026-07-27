// SPDX-License-Identifier: LGPL-3.0-or-later
#include "io/loaders/loader_registry.h"

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <string_view>

#include "io/loaders/dfd1/dfd1_loader.h"
#include "io/loaders/dfd2/dfd2_loader.h"
#include "io/loaders/fenix/fenix_loader.h"
#include "io/loaders/xplane12/xplane12_loader.h"

namespace bf {
namespace {

using FactoryFn = std::unique_ptr<Loader> (*)();

std::unique_ptr<Loader> MakeDfd1Loader() { return std::make_unique<Dfd1Loader>(); }

std::unique_ptr<Loader> MakeDfd2Loader() { return std::make_unique<Dfd2Loader>(); }

std::unique_ptr<Loader> MakeFenixLoader() { return std::make_unique<FenixLoader>(); }

std::unique_ptr<Loader> MakeXPlane12Loader() { return std::make_unique<XPlane12Loader>(); }

struct Entry {
  std::string_view name;
  FactoryFn factory;
};

// Sorted by name — enforced by the static_assert below. New loaders go here, in
// alphabetical order.
constexpr std::array kRegistry = {
    Entry{"dfd1", MakeDfd1Loader},
    Entry{"dfd2", MakeDfd2Loader},
    Entry{"fenix", MakeFenixLoader},
    Entry{"xplane12", MakeXPlane12Loader},
};

static_assert(std::is_sorted(kRegistry.begin(), kRegistry.end(),
                             [](const Entry& a, const Entry& b) { return a.name < b.name; }),
              "kRegistry entries must be sorted alphabetically by name");

}  // namespace

Result<std::unique_ptr<Loader>> MakeLoader(const std::string& name) {
  const auto it =
      std::lower_bound(kRegistry.begin(), kRegistry.end(), name,
                       [](const Entry& entry, const std::string& key) { return entry.name < key; });

  if (it == kRegistry.end() || it->name != name) {
    return Result<std::unique_ptr<Loader>>::Err(
        Error(ErrorCode::kInvalidArgument, "unknown loader: " + name));
  }
  return Result<std::unique_ptr<Loader>>::Ok(it->factory());
}

}  // namespace bf
