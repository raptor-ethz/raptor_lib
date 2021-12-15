#include "Participant.h"

bool raptor::Participant::check_for_data() {
    float x;

    for (int i = 0; i < 30; ++i) {
        x = pose_.pose.position.x;
    }
    if (x == 0.0) {
        std::cout   << "WARNING [Participant: " << id << "]: "
                    << "Bad data after initializiation of mocap_subscriber." 
                    << std::endl;
        return false;
    }
    return true;
}

const cpp_msg::Mocap& raptor::Participant::get_pose()
{
    return pose_;
}

const std::string& raptor::Participant::get_id()
{
    return id;
}