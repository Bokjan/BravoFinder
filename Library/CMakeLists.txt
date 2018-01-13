cmake_minimum_required(VERSION 3.0)
project(BravoFinderLibrary)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
    DataSet/DataSet.cpp
    DataSet/Leg.cpp
    DataStructure/Coordinate.cpp
    DataStructure/GraphHelper.cpp
    DataStructure/Vertex.cpp
    DataStructure/Graph.cpp
    )
set(
    HEADER_FILES
    DataSet/DataSet.hpp
    DataSet/Leg.hpp
    )
add_library(bfinder SHARED ${SOURCE_FILES})
set_target_properties(bfinder PROPERTIES VERSION 0.0.1 SOVERSION 0.0.1)
install(TARGETS bfinder LIBRARY DESTINATION lib)
install(FILES bfinder.h DESTINATION include/bfinder)
install(FILES ${HEADER_FILES} DESTINATION include/bfinder/DataSet)
