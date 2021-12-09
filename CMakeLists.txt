cmake_minimum_required(VERSION 3.16)
project(raptor_lib VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

add_library(raptor_lib)

target_sources(raptor_lib PRIVATE
    src/Participant.cpp
    src/Quad.cpp
    src/Item.cpp
)

# requirements
# find_package(MAVSDK REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

target_link_libraries(raptor_lib PRIVATE
comm_lib
yaml-cpp
)

target_include_directories(raptor_lib PUBLIC include)

add_subdirectory(communication_lib)