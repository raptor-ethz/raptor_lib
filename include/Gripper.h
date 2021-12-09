#pragma once

#include "Participant.h"

class Gripper : public raptor::Participant{
    public:

    Gripper(std::unique_ptr<DefaultParticipant> dp, 
            std::string sub_topic_name,
            std::string pub_topic_name);
    ~Gripper();

    DDSPublisher *position_pub;

    private:
    cpp_msg::QuadPositionCmd pos_cmd{};
};