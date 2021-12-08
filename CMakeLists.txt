cmake_minimum_required(VERSION 3.0.0)
project(raptor_lib VERSION 0.1.0)

add_library(raptor_lib INTERFACE)

# requirements
# find_package(MAVSDK REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

target_link_libraries(raptor_lib INTERFACE
# comm_lib
yaml-cpp
)

target_sources(raptor_lib INTERFACE
    ./include/Entity.h
)

add_subdirectory(include)
add_subdirectory(src)
# add_subdirectory(communication_lib)