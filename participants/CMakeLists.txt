#Participant
target_include_directories(raptor_lib PUBLIC Participant)
target_sources(raptor_lib PRIVATE
    Participant/Participant.cpp
    Participant/console.cpp)

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
    Quad/goToPos.cpp
    Quad/actions.cpp
    Quad/emergency.cpp
    Quad/swoop.cpp)