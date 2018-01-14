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
    DataSet/Result.cpp DataSet/Result.hpp)
add_library(bfinder SHARED ${SOURCE_FILES})
set_target_properties(bfinder PROPERTIES VERSION 0.0.1 SOVERSION 0.0.1)
install(TARGETS bfinder LIBRARY DESTINATION lib)
install(FILES bfinder.h DESTINATION include/bfinder)
install(FILES DataSet/DataSet.hpp DataSet/Leg.hpp DataSet/Result.hpp DESTINATION include/bfinder/DataSet)
install(FILES DataStructure/Coordinate.hpp DataStructure/Vertex.hpp DESTINATION include/bfinder/DataStructure)
