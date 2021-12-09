#pragma once

#include "Participant.h"

class Quad : public raptor::Participant{
    public:

    Quad(   std::unique_ptr<DefaultParticipant> dp, 
            std::string sub_topic_name,
            std::string pub_topic_name);
    ~Quad();

    DDSPublisher *position_pub;

    bool go_to_pos();

    private:
    cpp_msg::QuadPositionCmd pos_cmd{};
};