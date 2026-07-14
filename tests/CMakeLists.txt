include(Catch)

add_executable(bf_tests
               unit/coordinate_test.cc
               unit/result_test.cc
               unit/graph_test.cc
               unit/constraint_test.cc
               unit/yen_test.cc
               unit/cifp_parser_test.cc
               unit/loader_registry_test.cc
               unit/route_string_test.cc
               unit/route_parser_test.cc
               unit/route_json_test.cc
               unit/route_metrics_test.cc
               unit/small_vec_test.cc
               unit/bfdb_naming_test.cc
               unit/nav_detail_codec_test.cc
               integration/route_integration_test.cc
               integration/query_test.cc
               integration/dfd_loader_test.cc
               integration/graph_codec_test.cc
               integration/bfdb_inventory_test.cc
               integration/cifp_codec_test.cc
               integration/unified_cache_test.cc
               integration/mcp_registry_test.cc)

target_link_libraries(bf_tests PRIVATE bf::bravofinder3 bf_mcp_lib Catch2::Catch2WithMain)

# Register each Catch2 TEST_CASE with CTest. Run from the project root so the
# integration tests find the default navdata/ directory. Catch2 returns exit
# code 4 when a test SKIPs (e.g. real data absent); map it to CTest's skip so a
# skip is not reported as a failure.
#
# DISCOVERY_MODE PRE_TEST defers test enumeration to run time so the binary is
# not executed during the configure step (where ASan fails without LD_PRELOAD).
# When sanitizers are active the compiler links them dynamically; on this system
# the runtime lives under the GCC library directory rather than a standard
# search path. We ask the compiler for the exact path to libasan.so and wire it
# in via CROSSCOMPILING_EMULATOR (the hook catch_discover_tests uses as the
# executor for both the discovery run and each individual test invocation).
# When sanitizers are inactive (release builds) the compiler returns the bare
# filename "libasan.so", IS_ABSOLUTE is false, and we skip the property so
# tests run without any wrapper.
execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so
  OUTPUT_VARIABLE _asan_lib OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libtsan.so
  OUTPUT_VARIABLE _tsan_lib OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CMAKE_CXX_FLAGS MATCHES "-fsanitize=thread" AND IS_ABSOLUTE "${_tsan_lib}")
  set_property(TARGET bf_tests PROPERTY CROSSCOMPILING_EMULATOR
    "${CMAKE_COMMAND};-E;env;LD_PRELOAD=${_tsan_lib}")
elseif(CMAKE_CXX_FLAGS MATCHES "-fsanitize" AND IS_ABSOLUTE "${_asan_lib}")
  set_property(TARGET bf_tests PROPERTY CROSSCOMPILING_EMULATOR
    "${CMAKE_COMMAND};-E;env;LD_PRELOAD=${_asan_lib}")
endif()

catch_discover_tests(bf_tests
                     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
                     PROPERTIES SKIP_RETURN_CODE 4
                     DISCOVERY_MODE PRE_TEST)
