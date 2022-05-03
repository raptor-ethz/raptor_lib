#Participant
target_include_directories(raptor_lib PUBLIC Participant)
target_sources(raptor_lib PRIVATE
    Participant/Participant.cpp
    Participant/console.cpp)

#Gripper
target_include_directories(raptor_lib PUBLIC Gripper)
target_sources(raptor_lib PRIVATE
    Gripper/Gripper.cpp
    Gripper/console.cpp)

#Item
target_include_directories(raptor_lib PUBLIC Item)
target_sources(raptor_lib PRIVATE
    Item/Item.cpp)

#Obstacle
target_include_directories(raptor_lib PUBLIC Obstacle)
target_sources(raptor_lib PRIVATE
    Obstacle/Obstacle.cpp)

#a star
target_include_directories(raptor_lib PUBLIC Planner)
target_sources(raptor_lib PRIVATE
    Planner/Astar.cpp)

#Quad
target_include_directories(raptor_lib PUBLIC Quad)
target_sources(raptor_lib PRIVATE
    Quad/Quad.cpp
    Quad/goToPos.cpp
    Quad/actions.cpp
    Quad/emergency.cpp
    Quad/swoop.cpp)