# The bf command-line tool.
add_executable(bf main.cc)

target_link_libraries(bf PRIVATE bf::io CLI11::CLI11)
