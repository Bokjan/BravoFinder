# The bf-mcp-stdio local MCP server (stdio transport, hand-rolled JSON-RPC).
# The protocol layer, capabilities dressing, and entry point live here; the
# multi-cycle registry and the query handlers live in bf_query_lib, shared with
# the HTTP transport and the CLI. The library can be unit-tested without spawning
# the process; main.cc is a thin entry point that wires them together.
add_library(bf_mcp_lib STATIC mcp_server.cc tools.cc)
# bf_query_lib carries bf::bravofinder, rapidjson, and the pointer-overflow
# UBSan exemption as PUBLIC usage requirements, so they propagate here and to
# any test target that links this library.
target_link_libraries(bf_mcp_lib PUBLIC bf_query_lib)
target_include_directories(bf_mcp_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(bf_mcp_stdio main.cc)
target_link_libraries(bf_mcp_stdio PRIVATE bf_mcp_lib)

# External-facing name uses a hyphen (MCP ecosystem convention); the CMake
# target keeps the snake_case name required by Google Style.
set_target_properties(bf_mcp_stdio PROPERTIES OUTPUT_NAME "bf-mcp-stdio")
