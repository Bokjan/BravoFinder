# BravoFinder unified library: the graph + A* search (formerly core/) and the
# I/O layer (formerly io/) merged into a single static target bravofinder.
# Headers are included as "core/..." / "io/..." so the include root is this
# directory (engine/).
#
# sqlite3.c (the SQLite amalgamation) is compiled directly as a bravofinder
# source so the shipped libbravofinder.a is self-contained: sqlite3.o lives in
# the archive and no external bf::sqlite3 dependency leaks into the export set.
# (A separate OBJECT lib linked via $<TARGET_OBJECTS:> does NOT fold its objects
# into a static archive -- they are only pulled in at final executable link.)
#
# BRAVOFINDER_BENCH_VARIANT swaps the two search source files (and their headers)
# for a historical snapshot vendored under bench/variants/<v>/, so the benchmark
# can time baseline / +memoize / +Lawler / +workspace without any git checkout.
# Empty (the default) builds the current tree unchanged; it is only ever set in a
# throwaway build tree by bench/decompose.sh, never in a normal build.
set(BRAVOFINDER_BENCH_VARIANT "" CACHE STRING
    "Build bravofinder against a vendored search variant: baseline | memoize | lawler | workspace")

if(BRAVOFINDER_BENCH_VARIANT STREQUAL "")
  set(_bf_astar core/graph/astar.cc)
  set(_bf_yen   core/graph/yen_kshortest.cc)
else()
  set(_bf_variant_dir ${PROJECT_SOURCE_DIR}/bench/variants/${BRAVOFINDER_BENCH_VARIANT})
  if(NOT EXISTS ${_bf_variant_dir}/core/graph/astar.cc)
    message(FATAL_ERROR "Unknown BRAVOFINDER_BENCH_VARIANT '${BRAVOFINDER_BENCH_VARIANT}' "
                        "(expected baseline | memoize | lawler | workspace)")
  endif()
  set(_bf_astar ${_bf_variant_dir}/core/graph/astar.cc)
  set(_bf_yen   ${_bf_variant_dir}/core/graph/yen_kshortest.cc)
endif()

add_library(bravofinder STATIC
  # sqlite3.c is listed first (not last) so low-core machines (-j 4) start its
  # long -O3 compile at t=0 instead of queuing ~7s behind the ~23 C++ sources;
  # it is the single slowest TU, so an early start keeps it off the critical
  # path as much as possible (the -O3 cost itself is accepted, see below).
  ${_sqlite3_dir}/sqlite3.c
  core/domain/coordinate.cc
  core/domain/procedure.cc
  ${_bf_astar}
  ${_bf_yen}
  core/routing/route_parser.cc
  core/routing/route_string.cc
  io/loaders/sqlite_util.cc
  io/loaders/loader_registry.cc
  io/loaders/dfd1/dfd1_loader.cc
  io/loaders/dfd2/dfd2_loader.cc
  io/loaders/fenix/fenix_loader.cc
  io/loaders/xplane12/xplane12_loader.cc
  io/loaders/xplane12/cifp_parser.cc
  io/cache/graph_codec.cc
  io/cache/bfdb_inventory.cc
  io/cache/bfdb_naming.cc
  io/cache/cifp_codec.cc
  io/cache/nav_detail_codec.cc
  io/cache/unified_cache.cc
  io/graph_builder.cc
  io/procedure_connector.cc
  io/nav_database.cc
  io/nav_database_routing.cc
  io/nav_database_parse.cc
  io/nav_database_query.cc)

# When a variant is selected, its headers must win over engine/core/graph/*.h. A
# BEFORE include dir mirroring the "core/graph/" path shadows the two headers
# only. PRIVATE (not PUBLIC): the shadow is for compiling bravofinder's own
# variant sources; it must not leak into bravofinder's INTERFACE_INCLUDE_DIRECTORIES,
# or the export/install machinery rejects the source-tree-prefixed path.
if(NOT BRAVOFINDER_BENCH_VARIANT STREQUAL "")
  target_include_directories(bravofinder BEFORE PRIVATE ${_bf_variant_dir})
endif()

# Generate version.h from the project version (single source of truth). It lands
# in the binary dir; expose that dir so "core/version.h" resolves.
configure_file(core/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/core/version.h @ONLY)

# Include root is engine/ (build) so "#include core/..." and "#include io/..."
# resolve; the binary dir exposes the generated core/version.h. Install-tree
# include root is include/bf/ (see the install block below).
target_include_directories(bravofinder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include/bf>)

target_compile_features(bravofinder PUBLIC cxx_std_20)

# Scope the SQLite amalgamation's third-party-C options to sqlite3.c alone so
# bravofinder's own flags stay clean. -w silences all warnings (GCC/Clang; MSVC's
# C compile carries no /WX, so warnings are non-fatal there). SQLITE_* match the
# former standalone bf_sqlite3 target: load extension is never used; threadsafe=1
# (serialized default) plus per-connection SQLITE_OPEN_NOMUTEX (thread_local
# connections, never shared across threads) gives correct, lock-free reads.
#
# sqlite3.c stays at the config's full -O3/-O2 (not downgraded): it is the single
# slowest TU (~21s under release -O3 on one core), so to keep it off the
# parallel-build critical path on low-core machines it is listed FIRST in the
# source list above (starts at t=0 instead of queuing ~7s behind the ~23 C++
# sources). The -O3 compile cost itself is accepted.
set_source_files_properties(${_sqlite3_dir}/sqlite3.c PROPERTIES
  COMPILE_OPTIONS "$<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-w>"
  COMPILE_DEFINITIONS "SQLITE_OMIT_LOAD_EXTENSION;SQLITE_THREADSAFE=1")

# bravofinder's own compile of sqlite_util.cc pulls in <sqlite3.h> (via
# sqlite_util.h); this PRIVATE include (never in INSTALL_INTERFACE) points at the
# amalgamation dir. sqlite3.c itself reaches sqlite3.h the same way.
target_include_directories(bravofinder PRIVATE ${_sqlite3_dir})

# External-facing alias. Consumers (apps, tests, bench) link bf::bravofinder.
# EXPORT_NAME = the imported target name (bf::bravofinder); OUTPUT_NAME = the
# archive file name (libbravofinder.a on GCC/Clang, bravofinder.lib on MSVC).
set_target_properties(bravofinder PROPERTIES EXPORT_NAME bravofinder OUTPUT_NAME bravofinder)
add_library(bf::bravofinder ALIAS bravofinder)

# ---------- Install / export (SDK distribution) --------------------------------
# Public header allowlist: the FULL transitive include graph of nav_database.h
# (the SDK entry point) plus the generated version.h. Consumers #including
# nav_database.h reach every one of these, so all must ship. cifp_parser.h,
# pread_file.h, nav_data.h are "internal" by intent but exposed transitively by
# public headers; the clean fix (forward-declaring them away) is an API refactor
# beyond this milestone. Verified: none pulls in <sqlite3.h> (sqlite_util.h is
# NOT in the graph), so the SDK needs no sqlite3 headers.
if(BRAVOFINDER_INSTALL)
  install(TARGETS bravofinder EXPORT bravofinderTargets
          ARCHIVE DESTINATION lib)

  install(FILES core/result.h                          DESTINATION include/bf/core)
  install(FILES core/domain/airport.h                  DESTINATION include/bf/core/domain)
  install(FILES core/domain/airway.h                   DESTINATION include/bf/core/domain)
  install(FILES core/domain/coordinate.h               DESTINATION include/bf/core/domain)
  install(FILES core/domain/hold_fix.h                 DESTINATION include/bf/core/domain)
  install(FILES core/domain/ident.h                    DESTINATION include/bf/core/domain)
  install(FILES core/domain/mora_grid.h                DESTINATION include/bf/core/domain)
  install(FILES core/domain/msa.h                     DESTINATION include/bf/core/domain)
  install(FILES core/domain/navaid_detail.h            DESTINATION include/bf/core/domain)
  install(FILES core/domain/procedure.h                DESTINATION include/bf/core/domain)
  install(FILES core/domain/waypoint.h                 DESTINATION include/bf/core/domain)
  install(FILES core/query/query_types.h               DESTINATION include/bf/core/query)
  install(FILES core/routing/route.h                   DESTINATION include/bf/core/routing)
  install(FILES core/routing/route_request.h           DESTINATION include/bf/core/routing)
  install(FILES io/nav_database.h                     DESTINATION include/bf/io)
  install(FILES io/nav_data.h                         DESTINATION include/bf/io)
  install(FILES io/cache/cifp_codec.h                  DESTINATION include/bf/io/cache)
  install(FILES io/cache/nav_detail_codec.h            DESTINATION include/bf/io/cache)
  install(FILES io/cache/pread_file.h                 DESTINATION include/bf/io/cache)
  install(FILES io/loaders/xplane12/cifp_parser.h      DESTINATION include/bf/io/loaders/xplane12)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/core/version.h
          DESTINATION include/bf/core)
endif()
