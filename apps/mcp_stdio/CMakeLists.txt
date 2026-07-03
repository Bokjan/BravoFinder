# The bf-mcp-stdio local MCP server (stdio transport, hand-rolled JSON-RPC).
# main.cc is the entry point; mcp_server.cc is the protocol layer; tools.cc is
# the capability definitions.
add_executable(bf_mcp_stdio main.cc mcp_server.cc tools.cc)

target_link_libraries(bf_mcp_stdio PRIVATE bf::io rapidjson::rapidjson)

# External-facing name uses a hyphen (MCP ecosystem convention); the CMake
# target keeps the snake_case name required by Google Style.
set_target_properties(bf_mcp_stdio PROPERTIES OUTPUT_NAME "bf-mcp-stdio")

# rapidjson computes `nullPtr + offset` when its internal stack is first used,
# which UBSan's pointer-overflow check flags inside the header. The behavior is
# benign (0 + 0 pointer arithmetic), so narrow off just that one sub-check for
# this target; ASan and the rest of UBSan stay enabled. A no-op where the
# sanitizers are not active (release). GCC/Clang only; MSVC does not recognise
# this flag.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(bf_mcp_stdio PRIVATE -fno-sanitize=pointer-overflow)
endif()
