#pragma once

#include "Participant.h"

class Item : public raptor::Participant{
    public:

    Item(   std::string &raptor_participant_id, 
            std::unique_ptr<DefaultParticipant> &dp, 
            std::string &sub_topic_name);
    ~Item();
};