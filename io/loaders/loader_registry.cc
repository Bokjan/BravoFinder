#include "io/loaders/loader_registry.h"

#include <memory>
#include <string>

#include "io/loaders/xplane12/xplane12_loader.h"

namespace bf {

Result<std::unique_ptr<Loader>> MakeLoader(const std::string& name) {
  if (name == "xplane12") {
    return Result<std::unique_ptr<Loader>>::Ok(std::make_unique<XPlane12Loader>());
  }
  return Result<std::unique_ptr<Loader>>::Err(
      Error(ErrorCode::kInvalidArgument, "unknown loader: " + name));
}

}  // namespace bf
