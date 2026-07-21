# The bf command-line tool. A thin front-end over the shared query layer
# (bf_service_lib): it parses CLI flags, opens one NavDatabase, and delegates to a
# typed bf::service entry point that runs the engine call and renders the result
# in the requested format. The same entry points serve the MCP and HTTP
# transports; the CLI is their peer, not a separate query implementation.
add_executable(bf
               main.cc
               cli_common.cc
               cmd_build.cc
               cmd_route.cc
               cmd_query.cc
               cmd_parse_route.cc)

# bf_service_lib carries bf::bravofinder, rapidjson, and the rapidjson
# pointer-overflow UBSan exemption as PUBLIC usage requirements, so they
# propagate here; CLI11 is for argument parsing.
target_link_libraries(bf PRIVATE bf_service_lib CLI11::CLI11)
