#pragma once

#include "Participant.h"

class Gripper : public raptor::Participant {
 public:
  Gripper(std::string &raptor_participant_id,
          std::unique_ptr<DefaultParticipant> &dp, std::string &pub_topic_name);
  ~Gripper();

  DDSPublisher *grip_pub;
  void close();
  void open();

 private:
  cpp_msg::QuadPositionCmd grip_cmd{};
};