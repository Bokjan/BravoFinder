cmake_minimum_required(VERSION 3.21)

project(BravoFinder
        VERSION 3.0.0
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

# ---------------------------------------------------------------------------
# Dependencies (fetched at build time; not vendored into the repository)
# ---------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2)
FetchContent_MakeAvailable(CLI11)

# rapidjson is header-only; we use only its SAX-style Writer for JSON output.
# Disable its tests/examples/doc and expose the headers via an INTERFACE target,
# since upstream's own CMake config is dated.
FetchContent_Declare(
  rapidjson
  GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
  GIT_TAG v1.1.0)
set(RAPIDJSON_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(RAPIDJSON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RAPIDJSON_BUILD_DOC OFF CACHE BOOL "" FORCE)
FetchContent_GetProperties(rapidjson)
if(NOT rapidjson_POPULATED)
  FetchContent_Populate(rapidjson)
endif()
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

if(BRAVOFINDER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
