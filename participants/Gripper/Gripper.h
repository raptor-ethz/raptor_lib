#pragma once

#include "Participant.h"

class Gripper : public raptor::Participant {
 public:
  Gripper(const std::string &raptor_participant_id,
          std::unique_ptr<DefaultParticipant> &dp,
          const std::string &pub_topic_name);

  ~Gripper();

  DDSPublisher *grip_pub;

  // set angle from 0 to 70
  void set_angle_sym(int angle);
  void set_front_arm(int angle);
  void set_back_arm(int angle);

 private:
  cpp_msg::QuadPositionCmd grip_cmd{};
  const int max_angle = 80;
};