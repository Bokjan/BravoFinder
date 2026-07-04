cmake_minimum_required(VERSION 3.21)

project(BravoFinder
        VERSION 3.6.1
        DESCRIPTION "A realistic / compliant flight route engine"
        LANGUAGES CXX)

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
    GIT_TAG v3.5.4)
  FetchContent_MakeAvailable(Catch2)
  # Make Catch2's CTest integration module (Catch.cmake) available.
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(apps/cli)
add_subdirectory(apps/mcp_stdio)

if(BRAVOFINDER_BUILD_BENCH)
  add_subdirectory(bench)
endif()

if(BRAVOFINDER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
