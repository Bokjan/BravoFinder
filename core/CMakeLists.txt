# The core domain library: pure logic, no I/O framework dependencies.
add_library(bf_core
            domain/coordinate.cc)

# Headers are included as "core/..." so the include root is the project root.
target_include_directories(bf_core PUBLIC ${PROJECT_SOURCE_DIR})

target_compile_features(bf_core PUBLIC cxx_std_20)

# Provide a namespaced alias so consumers link bf::core.
add_library(bf::core ALIAS bf_core)
