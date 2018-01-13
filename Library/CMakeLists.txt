cmake_minimum_required(VERSION 3.0)
project(Library)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
    DataSet/DataSet.cpp
    DataStructure/Coordinate.cpp
    DataStructure/GraphHelper.cpp
    DataStructure/Vertex.cpp
    DataStructure/Graph.cpp
    DataSet/Leg.cpp)
add_library(bfinder SHARED ${SOURCE_FILES})
