# The bf-mcp local MCP server. Two transports share one Dispatcher: the stdio
# transport (hand-rolled JSON-RPC over stdin/stdout) and the MCP-over-HTTP
# transport (Streamable HTTP, built on the shared bf_http_server core, reused
# from the REST server). The transport-neutral dispatcher, both transport shells,
# the capabilities dressing, and entry point live here; the multi-cycle registry
# and the query handlers live in bf_service_lib. The library can be unit-tested
# without spawning the process; main.cc is a thin entry point that wires them up.
add_library(bf_mcp_lib STATIC dispatcher.cc tools.cc stdio_runner.cc mcp_http.cc)
# bf_service_lib carries bf::bravofinder, rapidjson, and the pointer-overflow
# UBSan exemption as PUBLIC usage requirements; bf_http_server carries the shared
# HTTP transport core (libuv + llhttp) used by the http transport. Both propagate
# here and to any test target that links this library.
target_link_libraries(bf_mcp_lib PUBLIC bf_service_lib bf_http_server)
target_include_directories(bf_mcp_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(bf_mcp main.cc)
target_link_libraries(bf_mcp PRIVATE bf_mcp_lib CLI11::CLI11)

# External-facing name uses a hyphen (MCP ecosystem convention); the CMake
# target keeps the snake_case name required by Google Style.
set_target_properties(bf_mcp PROPERTIES OUTPUT_NAME "bf-mcp")
