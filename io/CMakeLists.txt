# The I/O layer: data loaders, graph builder, and the NavDatabase facade.
add_library(bf_io
            loaders/loader_registry.cc
            loaders/xplane12/xplane12_loader.cc
            loaders/xplane12/cifp/cifp_parser.cc
            loaders/xplane12/cifp/procedure_connector.cc
            cache/graph_codec.cc
            cache/bfdb_inventory.cc
            cache/bfdb_naming.cc
            cache/cifp_codec.cc
            cache/nav_detail_codec.cc
            cache/unified_cache.cc
            graph_builder.cc
            nav_database.cc
            nav_database_routing.cc
            nav_database_parse.cc
            nav_database_query.cc)

target_include_directories(bf_io PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(bf_io PUBLIC bf::core)

add_library(bf::io ALIAS bf_io)
