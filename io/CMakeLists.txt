# The I/O layer: data loaders, graph builder, and the NavDatabase facade.
add_library(bf_io
            loaders/xplane/xplane_loader.cc
            loaders/xplane/cifp/cifp_parser.cc
            loaders/xplane/cifp/procedure_connector.cc
            cache/bfdb_cache.cc
            cache/bfdb_inventory.cc
            cache/bfdb_naming.cc
            cache/cifp_cache.cc
            cache/nav_detail_cache.cc
            graph_builder.cc
            nav_database.cc
            nav_database_routing.cc
            nav_database_parse.cc
            nav_database_query.cc)

target_include_directories(bf_io PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(bf_io PUBLIC bf::core)

add_library(bf::io ALIAS bf_io)
