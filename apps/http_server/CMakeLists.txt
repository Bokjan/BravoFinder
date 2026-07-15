# bf-http: the hand-rolled HTTP query server. A thin transport (libuv event loop
# + llhttp parser) over the shared bf_query_core handlers. The connection state
# machine, router, work offload, and listener live in a library so tests can
# start a real server on a loopback port; main.cc is a thin entry point.
#
# bf_query_core is linked PUBLIC, carrying bf::bravofinder3, rapidjson, and the
# rapidjson pointer-overflow UBSan exemption to consumers; libuv/llhttp are
# PUBLIC so the test target that links this lib inherits them too.
add_library(bf_http_lib STATIC conn.cc router.cc work.cc server.cc)
target_link_libraries(bf_http_lib PUBLIC bf_query_core uv_a llhttp::llhttp)
target_include_directories(bf_http_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(bf_http main.cc)
target_link_libraries(bf_http PRIVATE bf_http_lib CLI11::CLI11)

# External-facing name uses a hyphen (matching bf-mcp-stdio); the CMake target
# keeps the snake_case name required by Google Style.
set_target_properties(bf_http PROPERTIES OUTPUT_NAME "bf-http")
