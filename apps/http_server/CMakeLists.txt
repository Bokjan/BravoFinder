# bf-http: the hand-rolled HTTP query server. A thin transport (libuv event loop
# + llhttp parser) over the shared bf_query_core handlers. The query logic and
# the pointer-overflow UBSan exemption come transitively from bf_query_core
# (PUBLIC), so nothing rapidjson-related needs repeating here.
add_executable(bf_http main.cc conn.cc router.cc work.cc)
target_link_libraries(bf_http PRIVATE bf_query_core uv_a llhttp::llhttp CLI11::CLI11)
target_include_directories(bf_http PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# External-facing name uses a hyphen (matching bf-mcp-stdio); the CMake target
# keeps the snake_case name required by Google Style.
set_target_properties(bf_http PROPERTIES OUTPUT_NAME "bf-http")
