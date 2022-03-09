#include "Participant.h"

bool raptor::Participant::checkForData() {
    float x;

    for (int i = 0; i < 10; ++i) {
        mocap_sub_->listener->wait_for_data();
        x = pose_.pose.position.x;
    }
    if (x == 0.0) {
        std::cout   << "WARNING [Participant: " << id << "]: "
                    << "Bad data after initializiation of mocap_subscriber." 
                    << std::endl;
        return false;
    } else{
        std::cout   << "INFO [Participant: " << id << "]: "
                    << "Good data after initializiation of mocap_subscriber." 
                    << std::endl;
    }

    return true;
}

const cpp_msg::Mocap& raptor::Participant::getPose()
{
    return pose_;
}

const std::string& raptor::Participant::getId()
{
    return id;
}