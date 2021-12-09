#pragma once

#include "Participant.h"

class Item : public raptor::Participant{
    public:

    // Constructor
    Item(std::unique_ptr<DefaultParticipant> dp, std::string sub_topic_name);
};