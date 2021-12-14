#pragma once

#include "Participant.h"

class Item : public raptor::Participant{
    public:

    Item(std::unique_ptr<DefaultParticipant> dp, std::string sub_topic_name);
    ~Item();
};