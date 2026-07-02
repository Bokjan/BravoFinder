# The bf-mcp-stdio local MCP server (stdio transport, hand-rolled JSON-RPC).
# main.cc is the entry point; mcp_server.cc is the protocol layer; tools.cc is
# the capability definitions.
add_executable(bf_mcp_stdio main.cc mcp_server.cc tools.cc)

target_link_libraries(bf_mcp_stdio PRIVATE bf::io rapidjson::rapidjson)

# External-facing name uses a hyphen (MCP ecosystem convention); the CMake
# target keeps the snake_case name required by Google Style.
set_target_properties(bf_mcp_stdio PROPERTIES OUTPUT_NAME "bf-mcp-stdio")

# rapidjson 1.1.0 (latest tagged release, 2016) triggers two warnings under
# -Werror that are benign but can't be fixed without a newer release:
#   -Wnontrivial-memcall (Clang) / -Wclass-memaccess (GCC)
#     — internal memcpy over GenericValue types
#   -Wdeprecated-declarations
#     — internal use of std::iterator
# These fire from template instantiations in the user's translation unit, so
# SYSTEM includes don't help. Probe each flag and apply when available.
# Additionally, rapidjson computes `nullPtr + offset` on first stack use,
# tripping UBSan's pointer-overflow check — narrow off just that sub-check.
# All are GCC/Clang only; MSVC does not recognise these flags.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  include(CheckCXXCompilerFlag)

  target_compile_options(bf_mcp_stdio PRIVATE
    -fno-sanitize=pointer-overflow
    -Wno-deprecated-declarations)

  check_cxx_compiler_flag("-Wnontrivial-memcall" HAS_WNONTRIVIAL_MEMCALL)
  if(HAS_WNONTRIVIAL_MEMCALL)
    target_compile_options(bf_mcp_stdio PRIVATE -Wno-nontrivial-memcall)
  endif()

  check_cxx_compiler_flag("-Wclass-memaccess" HAS_WCLASS_MEMACCESS)
  if(HAS_WCLASS_MEMACCESS)
    target_compile_options(bf_mcp_stdio PRIVATE -Wno-class-memaccess)
  endif()
endif()
