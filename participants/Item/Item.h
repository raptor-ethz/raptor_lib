#pragma once

#include "Participant.h"

class Item : public raptor::Participant {
public:
  Item(const std::string &raptor_participant_id,
       std::unique_ptr<DefaultParticipant> &dp,
       const std::string &sub_topic_name);
  Item(const std::string &raptor_participant_id, std::string *const log,
       std::unique_ptr<DefaultParticipant> &dp,
       const std::string &sub_topic_name);
  ~Item();

  bool setInitialPosition();

  float getInitialPositionX();
  float getInitialPositionY();
  float getInitialPositionZ();

private:
  float initial_position_x_ = 0;
  float initial_position_y_ = 0;
  float initial_position_z_ = 1.5;
};