cmake_minimum_required(VERSION 3.0)
project(WebAPI)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
    main.cpp)

add_executable(bfweb ${SOURCE_FILES})
target_link_libraries(bfweb bfinder scaffold)
