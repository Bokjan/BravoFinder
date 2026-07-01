# The I/O layer: data loaders, graph builder, and the NavDatabase facade.
add_library(bf_io
            loaders/xplane/xplane_loader.cc
            loaders/xplane/cifp/cifp_parser.cc
            loaders/xplane/cifp/procedure_connector.cc
            cache/bfdb_cache.cc
            graph_builder.cc
            nav_database.cc)

target_include_directories(bf_io PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(bf_io PUBLIC bf::core)

add_library(bf::io ALIAS bf_io)
