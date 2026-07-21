# bf_query_lib: the transport-neutral query layer shared by the MCP and HTTP
# apps (and the CLI). It holds the multi-cycle NavDatabaseRegistry and the pure
# query handlers (find_routes / parse_route / the batch lookups), returning a
# JSON body plus an HTTP-style status. None of the consumers depend on each
# other; all depend here.
add_library(bf_query_lib STATIC handlers.cc queries.cc render.cc registry.cc)
target_link_libraries(bf_query_lib PUBLIC bf::bravofinder rapidjson::rapidjson)
target_include_directories(bf_query_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# rapidjson computes `nullPtr + offset` when its internal stack is first used,
# which UBSan's pointer-overflow check flags inside the header. The behavior is
# benign (0 + 0 pointer arithmetic), so narrow off just that one sub-check for
# the code that touches rapidjson; ASan and the rest of UBSan stay enabled. A
# no-op where the sanitizers are not active (release). GCC/Clang only; MSVC does
# not recognise this flag. PUBLIC so consumers (mcp/http libs, tests) that link
# this target and include rapidjson through it inherit the exemption.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(bf_query_lib PUBLIC -fno-sanitize=pointer-overflow)
endif()
