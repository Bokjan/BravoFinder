# BravoFinder unified library: the graph + A* search (formerly core/) and the
# I/O layer (formerly io/) merged into a single static target bf3. Headers are
# included as "core/..." / "io/..." so the include root is this directory (lib/).
#
# sqlite3.c (the SQLite amalgamation) is compiled directly as a bf3 source so the
# shipped libbravofinder3.a is self-contained: sqlite3.o lives in the archive and
# no external bf::sqlite3 dependency leaks into the export set. (A separate OBJECT
# lib linked via $<TARGET_OBJECTS:> does NOT fold its objects into a static
# archive -- they are only pulled in at final executable link.)
#
# BRAVOFINDER_BENCH_VARIANT swaps the two search source files (and their headers)
# for a historical snapshot vendored under bench/variants/<v>/, so the benchmark
# can time baseline / +memoize / +Lawler without any git checkout. Empty (the
# default) builds the current tree unchanged; it is only ever set in a throwaway
# build tree by bench/decompose.sh, never in a normal build.
set(BRAVOFINDER_BENCH_VARIANT "" CACHE STRING
    "Build bf3 against a vendored search variant: baseline | memoize | lawler")

if(BRAVOFINDER_BENCH_VARIANT STREQUAL "")
  set(_bf_astar core/graph/astar.cc)
  set(_bf_yen   core/graph/yen_kshortest.cc)
else()
  set(_bf_variant_dir ${PROJECT_SOURCE_DIR}/bench/variants/${BRAVOFINDER_BENCH_VARIANT})
  if(NOT EXISTS ${_bf_variant_dir}/core/graph/astar.cc)
    message(FATAL_ERROR "Unknown BRAVOFINDER_BENCH_VARIANT '${BRAVOFINDER_BENCH_VARIANT}' "
                        "(expected baseline | memoize | lawler)")
  endif()
  set(_bf_astar ${_bf_variant_dir}/core/graph/astar.cc)
  set(_bf_yen   ${_bf_variant_dir}/core/graph/yen_kshortest.cc)
endif()

add_library(bf3 STATIC
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
  io/loaders/xplane12/xplane12_loader.cc
  io/loaders/xplane12/cifp/cifp_parser.cc
  io/loaders/xplane12/cifp/procedure_connector.cc
  io/cache/graph_codec.cc
  io/cache/bfdb_inventory.cc
  io/cache/bfdb_naming.cc
  io/cache/cifp_codec.cc
  io/cache/nav_detail_codec.cc
  io/cache/unified_cache.cc
  io/graph_builder.cc
  io/nav_database.cc
  io/nav_database_routing.cc
  io/nav_database_parse.cc
  io/nav_database_query.cc
  ${_sqlite3_dir}/sqlite3.c)

# When a variant is selected, its headers must win over lib/core/graph/*.h. A
# BEFORE include dir mirroring the "core/graph/" path shadows the two headers
# only.
if(NOT BRAVOFINDER_BENCH_VARIANT STREQUAL "")
  target_include_directories(bf3 BEFORE PUBLIC ${_bf_variant_dir})
endif()

# Generate version.h from the project version (single source of truth). It lands
# in the binary dir; expose that dir so "core/version.h" resolves.
configure_file(core/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/core/version.h @ONLY)

# Include root is lib/ (build) so "#include core/..." and "#include io/..."
# resolve; the binary dir exposes the generated core/version.h. Install-tree
# include root is include/bf/ (added in the SDK export milestone).
target_include_directories(bf3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include/bf>)

target_compile_features(bf3 PUBLIC cxx_std_20)

# Scope the SQLite amalgamation's third-party-C options to sqlite3.c alone so
# bf3's own flags stay clean. -w silences all warnings (GCC/Clang; MSVC's C
# compile carries no /WX, so warnings are non-fatal there). SQLITE_* match the
# former standalone bf_sqlite3 target: load extension is never used; threadsafe=1
# (serialized default) plus per-connection SQLITE_OPEN_NOMUTEX (thread_local
# connections, never shared across threads) gives correct, lock-free reads.
set_source_files_properties(${_sqlite3_dir}/sqlite3.c PROPERTIES
  COMPILE_OPTIONS "$<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>>:-w>"
  COMPILE_DEFINITIONS "SQLITE_OMIT_LOAD_EXTENSION;SQLITE_THREADSAFE=1")

# bf3's own compile of sqlite_util.cc pulls in <sqlite3.h> (via sqlite_util.h);
# this PRIVATE include (never in INSTALL_INTERFACE) points at the amalgamation
# dir. sqlite3.c itself reaches sqlite3.h the same way.
target_include_directories(bf3 PRIVATE ${_sqlite3_dir})

# External-facing alias. Consumers (apps, tests, bench) link bf::bravofinder3.
set_target_properties(bf3 PROPERTIES EXPORT_NAME bravofinder3)
add_library(bf::bravofinder3 ALIAS bf3)
