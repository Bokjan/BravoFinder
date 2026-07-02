# The bf-mcp-stdio local MCP server (stdio transport, hand-rolled JSON-RPC).
# main.cc is the entry point; mcp_server.cc is the protocol layer; tools.cc is
# the capability definitions.
add_executable(bf_mcp_stdio main.cc mcp_server.cc tools.cc)

target_link_libraries(bf_mcp_stdio PRIVATE bf::io rapidjson::rapidjson)

# External-facing name uses a hyphen (MCP ecosystem convention); the CMake
# target keeps the snake_case name required by Google Style.
set_target_properties(bf_mcp_stdio PROPERTIES OUTPUT_NAME "bf-mcp-stdio")

# Same benign rapidjson pointer-overflow suppression as the CLI target, plus
# two more: rapidjson 1.1.0's internal memcpy over GenericValue trips
# -Wnontrivial-memcall, and its use of the deprecated std::iterator trips
# -Wdeprecated-declarations, both under -Werror with a modern libc++. The
# behaviors are benign and there is no newer tagged release to move to, so
# narrow off just these warnings for this target (no-ops where warnings are not
# active, e.g. release, or when the compiler is not GCC/Clang).
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-Wnontrivial-memcall" HAS_WNONTRIVIAL_MEMCALL)

  target_compile_options(bf_mcp_stdio PRIVATE
    -fno-sanitize=pointer-overflow
    -Wno-error=deprecated-declarations)
  if(HAS_WNONTRIVIAL_MEMCALL)
    target_compile_options(bf_mcp_stdio PRIVATE -Wno-error=nontrivial-memcall)
  endif()
endif()
