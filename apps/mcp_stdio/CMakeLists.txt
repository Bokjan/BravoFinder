# The bf-mcp-stdio local MCP server (stdio transport, hand-rolled JSON-RPC).
# The protocol layer, capabilities, and multi-cycle registry live in a library
# so they can be unit-tested without spawning the process; main.cc is a thin
# entry point that wires them together.
add_library(bf_mcp_lib STATIC mcp_server.cc tools.cc registry.cc)
target_link_libraries(bf_mcp_lib PUBLIC bf::bravofinder3 rapidjson::rapidjson)
target_include_directories(bf_mcp_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(bf_mcp_stdio main.cc)
target_link_libraries(bf_mcp_stdio PRIVATE bf_mcp_lib)

# External-facing name uses a hyphen (MCP ecosystem convention); the CMake
# target keeps the snake_case name required by Google Style.
set_target_properties(bf_mcp_stdio PROPERTIES OUTPUT_NAME "bf-mcp-stdio")

# rapidjson computes `nullPtr + offset` when its internal stack is first used,
# which UBSan's pointer-overflow check flags inside the header. The behavior is
# benign (0 + 0 pointer arithmetic), so narrow off just that one sub-check for
# the code that touches rapidjson; ASan and the rest of UBSan stay enabled. A
# no-op where the sanitizers are not active (release). GCC/Clang only; MSVC does
# not recognise this flag. PUBLIC so test targets linking the library inherit it.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(bf_mcp_lib PUBLIC -fno-sanitize=pointer-overflow)
endif()
