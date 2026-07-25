# bf-http: the hand-rolled HTTP query server. A thin REST router over the shared
# bf_service_lib handlers, carried by the shared bf_http_server transport core
# (libuv event loop + llhttp parser + hardening). Only the router lives here now;
# the connection state machine, listener, and work offload live in
# libs/http_server/, shared with the MCP-over-HTTP transport (apps/mcp).
#
# bf_service_lib is linked PUBLIC, carrying bf::bravofinder, rapidjson, and the
# rapidjson pointer-overflow UBSan exemption to consumers; bf_http_server is
# PUBLIC so the test target that links this lib inherits libuv/llhttp and the
# transport headers too.
add_library(bf_http_lib STATIC router.cc)
target_link_libraries(bf_http_lib PUBLIC bf_service_lib bf_http_server)
target_include_directories(bf_http_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(bf_http main.cc)
target_link_libraries(bf_http PRIVATE bf_http_lib CLI11::CLI11)

# External-facing name uses a hyphen (matching bf-mcp); the CMake target
# keeps the snake_case name required by Google Style.
set_target_properties(bf_http PROPERTIES OUTPUT_NAME "bf-http")
