# Header-only status-code constants (http_status.h). Linked by bf_http_server
# and by bf_service_lib so HandlerResult.status and the transport share one
# source without pulling libuv into the CLI (which links service, not the
# full HTTP server).
add_library(bf_http_status INTERFACE)
add_library(bf::http_status ALIAS bf_http_status)
target_include_directories(bf_http_status INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

# bf_http_server: the shared hand-rolled HTTP/1.1 transport core (a libuv event
# loop + the llhttp parser + the safety hardening llhttp does not do), factored
# out of apps/http so both the REST server (apps/http) and the MCP-over-HTTP
# transport (apps/mcp) can reuse it. It is deliberately free of any query /
# JSON-schema knowledge -- it defines the RequestHandler interface and the
# WorkResult value type, and a consumer gives requests meaning -- so it does NOT
# depend on bf_service_lib (keeping the transport core independent of the query
# layer, mirroring how libs/engine/ stays JSON-/network-free).
add_library(bf_http_server STATIC conn.cc server.cc work.cc)
# libuv + llhttp headers appear in the public headers (conn.h / server.h /
# work.h), so link them PUBLIC to propagate to consumers. rapidjson is used only
# in conn.cc (the JsonError helper for transport-level rejects), so it is PRIVATE.
target_link_libraries(bf_http_server PUBLIC uv_a llhttp::llhttp bf::http_status)
target_link_libraries(bf_http_server PRIVATE rapidjson::rapidjson)
target_include_directories(bf_http_server PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# conn.cc touches rapidjson, which trips UBSan's pointer-overflow sub-check on a
# benign `nullPtr + 0` inside the header (see libs/service/CMakeLists.txt). Narrow off
# just that one sub-check for this library's TUs; ASan and the rest of UBSan stay
# on. A no-op in release / on MSVC.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(bf_http_server PRIVATE -fno-sanitize=pointer-overflow)
endif()
