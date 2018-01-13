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
file(GLOB HEADER_FILES "*.hpp")
add_library(bfinder SHARED ${SOURCE_FILES})
install(TARGETS bfinder LIBRARY DESTINATION lib)
install(FILES bfinder.h ${HEADER_FILES} DESTINATION include/bfinder)
