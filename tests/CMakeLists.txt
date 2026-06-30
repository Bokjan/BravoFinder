include(Catch)

add_executable(bf_tests
               unit/coordinate_test.cc
               unit/result_test.cc
               unit/graph_test.cc
               integration/route_integration_test.cc)

target_link_libraries(bf_tests PRIVATE bf::io Catch2::Catch2WithMain)

# Register each Catch2 TEST_CASE with CTest. Run from the project root so the
# integration tests find the default navdata/ directory.
catch_discover_tests(bf_tests WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
