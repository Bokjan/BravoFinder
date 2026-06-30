include(Catch)

add_executable(bf_tests
               unit/coordinate_test.cc
               unit/result_test.cc)

target_link_libraries(bf_tests PRIVATE bf::core Catch2::Catch2WithMain)

# Register each Catch2 TEST_CASE with CTest.
catch_discover_tests(bf_tests)
