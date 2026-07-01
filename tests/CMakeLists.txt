include(Catch)

add_executable(bf_tests
               unit/coordinate_test.cc
               unit/result_test.cc
               unit/graph_test.cc
               unit/constraint_test.cc
               unit/yen_test.cc
               unit/cifp_parser_test.cc
               integration/route_integration_test.cc
               integration/bfdb_cache_test.cc)

target_link_libraries(bf_tests PRIVATE bf::io Catch2::Catch2WithMain)

# Register each Catch2 TEST_CASE with CTest. Run from the project root so the
# integration tests find the default navdata/ directory. Catch2 returns exit
# code 4 when a test SKIPs (e.g. real data absent); map it to CTest's skip so a
# skip is not reported as a failure.
catch_discover_tests(bf_tests
                     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                     PROPERTIES SKIP_RETURN_CODE 4)
