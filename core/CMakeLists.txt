# The graph + A* search live in core (pure logic, no I/O).
add_library(bf_core
            domain/coordinate.cc
            domain/procedure.cc
            graph/astar.cc
            graph/yen_kshortest.cc
            routing/route_parser.cc
            routing/route_string.cc)

# Generate version.h from the project version (single source of truth). It lands
# in the binary dir; expose that dir so "core/version.h" resolves.
configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/core/version.h @ONLY)

# Headers are included as "core/..." so the include root is the project root.
target_include_directories(bf_core PUBLIC ${PROJECT_SOURCE_DIR}
                                          ${CMAKE_CURRENT_BINARY_DIR})

target_compile_features(bf_core PUBLIC cxx_std_20)

# Provide a namespaced alias so consumers link bf::core.
add_library(bf::core ALIAS bf_core)
