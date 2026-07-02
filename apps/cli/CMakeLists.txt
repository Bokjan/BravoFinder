# The bf command-line tool.
add_executable(bf
               main.cc
               cli_common.cc
               cmd_build.cc
               cmd_route.cc
               cmd_query.cc
               cmd_parse_route.cc)

target_link_libraries(bf PRIVATE bf::io CLI11::CLI11 rapidjson::rapidjson)

# rapidjson 1.1.0 (the latest release, 2016) computes `nullPtr + offset` when its
# internal stack is first used, which UBSan's pointer-overflow check flags inside
# the header. The behavior is benign and there is no newer tagged release to move
# to, so narrow off just that one sub-check for this target; ASan and the rest of
# UBSan stay enabled. A no-op where the sanitizers are not active (release).
# GCC/Clang only; MSVC does not recognise this flag.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(bf PRIVATE -fno-sanitize=pointer-overflow)
endif()
