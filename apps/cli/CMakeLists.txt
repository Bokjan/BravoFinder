# The bf command-line tool. A thin front-end over the shared query layer
# (bf_service_lib): it parses CLI flags, opens one NavDatabase, and delegates to a
# typed bf::service entry point that runs the engine call and renders the result
# in the requested format. The same entry points serve the MCP and HTTP
# transports; the CLI is their peer, not a separate query implementation.
#
# The shared helpers live in a library, like bf_mcp_lib / bf_http_lib, so the
# argument-spec parsers (--alt, --airway-filter) can be unit-tested directly
# rather than only through the assembled binary.
add_library(bf_cli_lib STATIC cli_common.cc)
# bf_service_lib carries bf::bravofinder, rapidjson, and the rapidjson
# pointer-overflow UBSan exemption as PUBLIC usage requirements, so they
# propagate here and to any test target that links this library.
target_link_libraries(bf_cli_lib PUBLIC bf_service_lib)
target_include_directories(bf_cli_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(bf
               main.cc
               cmd_build.cc
               cmd_route.cc
               cmd_query.cc
               cmd_parse_route.cc)

# CLI11 is for argument parsing; the rest propagates from bf_cli_lib.
target_link_libraries(bf PRIVATE bf_cli_lib CLI11::CLI11)
