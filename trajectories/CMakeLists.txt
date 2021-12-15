target_include_directories(raptor_lib PUBLIC minimum_jerk)
target_include_directories(raptor_lib PUBLIC includes)

target_sources(raptor_lib PRIVATE
    minimum_jerk/RapidTrajectoryGenerator.cpp
    minimum_jerk/SingleAxisTrajectory.cpp)