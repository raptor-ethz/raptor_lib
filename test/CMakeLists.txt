add_executable(raptor_lib_test main.cpp)

target_link_libraries(raptor_lib_test PRIVATE
raptor_lib
comm_lib
yaml-cpp
)