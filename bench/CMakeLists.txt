# Route-search microbenchmark. Not built by default -- it is a developer tool for
# reproducing the numbers in docs/performance.zh-CN.md, not a shipped artifact.
# Enable explicitly:  cmake --preset release -DBRAVOFINDER_BUILD_BENCH=ON
add_executable(bf_route_bench route_bench.cc)
target_link_libraries(bf_route_bench PRIVATE bf::bravofinder3)
