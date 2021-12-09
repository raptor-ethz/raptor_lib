#pragma once

#include "Participant.h"

class Quad : public raptor::Participant{
    public:

    // Constructor
    Quad(   std::unique_ptr<DefaultParticipant> dp, 
            std::string sub_topic_name,
            std::string pos_pub_topic_name);

    DDSPublisher *position_pub;

    bool go_to_pos();

    private:
    cpp_msg::QuadPositionCmd pos_cmd{};
};