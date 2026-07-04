# The graph + A* search live in core (pure logic, no I/O).
#
# BRAVOFINDER_BENCH_VARIANT swaps the two search source files (and their headers)
# for a historical snapshot vendored under bench/variants/<v>/, so the benchmark
# can time baseline / +memoize / +Lawler without any git checkout. Empty (the
# default) builds the current tree unchanged; it is only ever set in a throwaway
# build tree by bench/decompose.sh, never in a normal build.
set(BRAVOFINDER_BENCH_VARIANT "" CACHE STRING
    "Build bf_core against a vendored search variant: baseline | memoize | lawler")

if(BRAVOFINDER_BENCH_VARIANT STREQUAL "")
  set(_bf_astar graph/astar.cc)
  set(_bf_yen graph/yen_kshortest.cc)
else()
  set(_bf_variant_dir ${PROJECT_SOURCE_DIR}/bench/variants/${BRAVOFINDER_BENCH_VARIANT})
  if(NOT EXISTS ${_bf_variant_dir}/core/graph/astar.cc)
    message(FATAL_ERROR "Unknown BRAVOFINDER_BENCH_VARIANT '${BRAVOFINDER_BENCH_VARIANT}' "
                        "(expected baseline | memoize | lawler)")
  endif()
  set(_bf_astar ${_bf_variant_dir}/core/graph/astar.cc)
  set(_bf_yen ${_bf_variant_dir}/core/graph/yen_kshortest.cc)
endif()

add_library(bf_core
            domain/coordinate.cc
            domain/procedure.cc
            ${_bf_astar}
            ${_bf_yen}
            routing/route_parser.cc
            routing/route_string.cc)

# When a variant is selected, its headers must win over core/graph/*.h. A BEFORE
# include dir mirroring the "core/graph/" path shadows the two headers only.
if(NOT BRAVOFINDER_BENCH_VARIANT STREQUAL "")
  target_include_directories(bf_core BEFORE PUBLIC ${_bf_variant_dir})
endif()

# Generate version.h from the project version (single source of truth). It lands
# in the binary dir; expose that dir so "core/version.h" resolves.
configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/core/version.h @ONLY)

# Headers are included as "core/..." so the include root is the project root.
target_include_directories(bf_core PUBLIC ${PROJECT_SOURCE_DIR}
                                          ${CMAKE_CURRENT_BINARY_DIR})

target_compile_features(bf_core PUBLIC cxx_std_20)

# Provide a namespaced alias so consumers link bf::core.
add_library(bf::core ALIAS bf_core)
