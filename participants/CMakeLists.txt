#Participant
target_include_directories(raptor_lib PUBLIC Participant)
target_sources(raptor_lib PRIVATE
    Participant/Participant.cpp)

#Gripper
target_include_directories(raptor_lib PUBLIC Gripper)
target_sources(raptor_lib PRIVATE
    Gripper/Gripper.cpp)

#Item
target_include_directories(raptor_lib PUBLIC Item)
target_sources(raptor_lib PRIVATE
    Item/Item.cpp)

#Quad
target_include_directories(raptor_lib PUBLIC Quad)
target_sources(raptor_lib PRIVATE
    Quad/Quad.cpp
    Quad/go_to_pos.cpp
    Quad/actions.cpp
    Quad/swoop.cpp)

#Logger
target_include_directories(raptor_lib PUBLIC logger)
target_sources(raptor_lib PRIVATE
    logger/logger.cpp)