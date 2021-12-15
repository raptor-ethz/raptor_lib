#pragma once

#include "Participant.h"

class Item : public raptor::Participant{
    public:

    Item(   const std::string &raptor_participant_id, 
            std::unique_ptr<DefaultParticipant> &dp, 
            const std::string &sub_topic_name);
    ~Item();
};