cmake_minimum_required(VERSION 3.21)

project(BravoFinder
        VERSION 3.18.17
        DESCRIPTION "A realistic and compliant flight routing engine and its query tools"
        LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# Global settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile_commands.json for editors / clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Treat this as the top-level project only when not embedded elsewhere.
if(PROJECT_IS_TOP_LEVEL)
  option(BRAVOFINDER_BUILD_TESTS "Build BravoFinder tests" ON)
else()
  option(BRAVOFINDER_BUILD_TESTS "Build BravoFinder tests" OFF)
endif()

# The performance microbenchmark (bench/) is a developer tool, off by default so
# it never pollutes normal builds, tests, or CI. Enable with
# -DBRAVOFINDER_BUILD_BENCH=ON to reproduce docs/performance.zh-CN.md.
option(BRAVOFINDER_BUILD_BENCH "Build the route-search microbenchmark" OFF)

# Install the headers + library only when BravoFinder is the top-level project
# (FetchContent / add_subdirectory consumers do not trigger install). Matches
# the BRAVOFINDER_BUILD_TESTS idiom above (PROJECT_IS_TOP_LEVEL, not the older
# CMAKE_SOURCE_DIR comparison).
if(PROJECT_IS_TOP_LEVEL)
  option(BRAVOFINDER_INSTALL "Install BravoFinder headers and libraries" ON)
else()
  set(BRAVOFINDER_INSTALL OFF)
endif()

# ---------------------------------------------------------------------------
# Compile-time source-path sanitization
# ---------------------------------------------------------------------------
# Strip the absolute repo prefix from __FILE__ (and std::source_location, which
# reads the same __FILE__) at compile time so log lines show short relative
# paths like "io/loaders/fenix/fenix_loader.cc" instead of a machine-specific
# absolute path. Applied globally (not bravofinder-only): once the log macros
# exist, apps/tests will use BF_LOG_* too, and their __FILE__ must trim
# consistently. sqlite3.c / libuv / llhttp also pick this up but never log, so
# it is harmless there.
if(MSVC)
  if(MSVC_VERSION GREATER_EQUAL 1927)
    file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/" _bf_repo_native)
    add_compile_options("/d1trimfile:${_bf_repo_native}")
  endif()
  # /Zc:preprocessor enables the conforming preprocessor (VS2019 16.5+ / MSVC
  # 19.25+, CI matrix satisfies it; loadmaster uses the same scheme). Required
  # by BF_LOG_* macros' __VA_OPT__ (core/log.h). Applied globally (not
  # bravofinder-only): any TU including log.h (bravofinder sources, apps, tests)
  # uses __VA_OPT__, so the flag must reach every consumer. Scoping it
  # per-target is how bf_tests was missed originally -- it uses BF_LOG_* but had
  # no flag, breaking the Windows CI leg.
  if(MSVC_VERSION GREATER_EQUAL 1925)
    add_compile_options("/Zc:preprocessor")
  endif()
else()
  add_compile_options("-ffile-prefix-map=${CMAKE_SOURCE_DIR}/=")
endif()

# ---------------------------------------------------------------------------
# Dependencies (fetched at build time; not vendored into the repository)
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.6.2)
FetchContent_MakeAvailable(CLI11)

# rapidjson is header-only; we use only its SAX-style Writer and DOM parser.
# SOURCE_SUBDIR points at a nonexistent directory so FetchContent_MakeAvailable
# only downloads the sources without running upstream's CMake (which would pull
# in rapidjson's own examples/tests/docs targets). We expose the headers
# ourselves via an INTERFACE target.
FetchContent_Declare(
  rapidjson
  GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
  GIT_TAG 24b5e7a8b27f42fa16b96fc70aade9106cf7102f
  SOURCE_SUBDIR do-not-build)
FetchContent_MakeAvailable(rapidjson)
add_library(rapidjson INTERFACE)
target_include_directories(rapidjson INTERFACE SYSTEM ${rapidjson_SOURCE_DIR}/include)
add_library(rapidjson::rapidjson ALIAS rapidjson)

if(BRAVOFINDER_BUILD_TESTS)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.15.2)
  FetchContent_MakeAvailable(Catch2)
  # Make Catch2's CTest integration module (Catch.cmake) available.
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

# SQLite amalgamation (read-only, used by the DFD loaders). A single ~9 MB C
# translation unit, compiled once into its own static lib so (a) it is not
# recompiled when consumers change and (b) its third-party C warnings can be
# silenced without polluting the project's -Werror. Pin a specific version via
# URL_HASH; bumping SQLite means updating URL + hash together.
FetchContent_Declare(
  sqlite3
  URL https://www.sqlite.org/2024/sqlite-amalgamation-3460000.zip
  URL_HASH SHA256=712a7d09d2a22652fb06a49af516e051979a3984adb067da86760e60ed51a7f5
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(sqlite3)

# SQLite amalgamation (read-only, used by the DFD loaders). Compiled directly as
# a bravofinder source (libs/engine/CMakeLists.txt adds ${_sqlite3_dir}/sqlite3.c), so
# the shipped libbravofinder.a is self-contained -- sqlite3.o lives in the
# archive with no external bf::sqlite3 dependency leaking into the export set.
# (A separate OBJECT lib linked via $<TARGET_OBJECTS:> does NOT fold its objects
# into a static archive: they are only pulled in at final executable link,
# leaving the archive with undefined sqlite3 symbols.) Third-party-C warnings and
# SQLITE_* defines are scoped to sqlite3.c via set_source_files_properties in
# libs/engine/CMakeLists.txt.
set(_sqlite3_dir ${sqlite3_SOURCE_DIR})

# libuv (event-loop async I/O + a built-in worker threadpool) and llhttp (Node's
# HTTP/1.1 parser): the two C libraries behind the hand-rolled HTTP query server
# (apps/http). Both are proper CMake projects with native MSVC support;
# libuv abstracts epoll/kqueue/IOCP under one API. We build only the static
# libraries, no tests/benchmarks. As C libraries they are untouched by the
# project's C++ -Werror (CMAKE_C_FLAGS is unset, exactly like the SQLite
# amalgamation); we additionally mark their headers SYSTEM below so the C++ that
# includes them stays clean under -Wpedantic -Werror. Pin exact tags.
#
# Force static for BOTH C libs up front: set BUILD_SHARED_LIBS OFF before either
# FetchContent_MakeAvailable so libuv also honors it (it builds only uv_a, the
# static target we consume, and skips the shared uv target). Setting it only
# before llhttp -- as this used to -- left libuv's static-only behavior implicit
# and order-fragile.
#
# Must be a CACHE var (with FORCE): llhttp's CMakeLists uses
# `option(BUILD_SHARED_LIBS "..." ON)`, and option() only consults the CACHE.
# A plain `set(... OFF)` (non-cache) is invisible to that option(), so it falls
# back to its default ON -- building llhttp_shared too. Both variants then emit
# to the same Debug llhttp.pdb and race under /MP (MSVC C1041). CACHE+FORCE makes
# option() see the OFF and skip the shared target (we only consume llhttp_static).
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(LIBUV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LIBUV_BUILD_BENCH OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  libuv
  GIT_REPOSITORY https://github.com/libuv/libuv.git
  GIT_TAG v1.49.2)
FetchContent_MakeAvailable(libuv)

# llhttp: build the static library and alias it as llhttp::llhttp (its CMake
# points the alias at llhttp_static when the shared build is off; BUILD_SHARED_LIBS
# is already OFF from above).
set(BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  llhttp
  GIT_REPOSITORY https://github.com/nodejs/llhttp.git
  GIT_TAG release/v9.2.1)
FetchContent_MakeAvailable(llhttp)

# Present the two C libraries' interface headers as system headers, so including
# uv.h / llhttp.h from the -Wall -Wextra -Wpedantic -Werror C++ code never trips
# a warning inside third-party headers.
foreach(_c_lib uv_a llhttp_static)
  if(TARGET ${_c_lib})
    get_target_property(_c_incs ${_c_lib} INTERFACE_INCLUDE_DIRECTORIES)
    if(_c_incs)
      set_target_properties(${_c_lib} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_c_incs}")
    endif()
    # MSVC /MP compiles these C libraries' several .c files in parallel sharing
    # one .pdb; without /FS a cl.exe that cannot grab the pdb lock fails with
    # C1041 (an intermittent, timing-dependent build flake -- see windows-debug).
    # /FS routes pdb writes through MSPDBSRV.EXE so the writers queue instead of
    # erroring. No-op off MSVC.
    if(MSVC)
      target_compile_options(${_c_lib} PRIVATE /FS)
    endif()
  endif()
endforeach()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(libs/engine)
add_subdirectory(libs/service)
add_subdirectory(libs/http_server)
add_subdirectory(apps/cli)
add_subdirectory(apps/mcp)
add_subdirectory(apps/http)

if(BRAVOFINDER_BUILD_BENCH)
  add_subdirectory(bench)
endif()

if(BRAVOFINDER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Install / export (SDK distribution)
# ---------------------------------------------------------------------------
if(BRAVOFINDER_INSTALL)
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    cmake/bravofinderConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/bravofinderConfig.cmake
    INSTALL_DESTINATION lib/cmake/bravofinder)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/bravofinderConfig.cmake
          DESTINATION lib/cmake/bravofinder)
  install(EXPORT bravofinderTargets
          FILE bravofinderTargets.cmake
          NAMESPACE bf::
          DESTINATION lib/cmake/bravofinder)
  install(FILES LICENSE.md LICENSE.MIT THIRD_PARTY_LICENSES.md
          DESTINATION share/doc/BravoFinder)
  # libs/engine is LGPL-3.0 (statically linked into the binaries). Rename its
  # so it does not collide with the root overview LICENSE.md at the same
  # DESTINATION; its full GPLv3 text ships alongside as LICENSE.GPLv3.
  install(FILES libs/engine/LICENSE RENAME LICENSE.LGPLv3
          DESTINATION share/doc/BravoFinder)
  install(FILES libs/engine/LICENSE.GPLv3
          DESTINATION share/doc/BravoFinder)
endif()
