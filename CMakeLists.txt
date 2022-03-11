cmake_minimum_required(VERSION 3.16)
project(raptor_lib VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

add_library(raptor_lib)

# requirements
# find_package(MAVSDK REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

target_link_libraries(raptor_lib PRIVATE
comm_lib
yaml-cpp
)

add_subdirectory(lib)
add_subdirectory(participants)
add_subdirectory(logger)
add_subdirectory(trajectories)
add_subdirectory(test)