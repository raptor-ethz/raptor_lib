#pragma once

#include "RotGripCmd_msg.h"
#include "RotGripCmd_msgPubSubTypes.h"
#include "domain_participant.h"
#include "publisher.h"

class Gripper {
public:
  Gripper(const std::string &raptor_participant_id,
          std::unique_ptr<DefaultParticipant> &dp,
          const std::string &pub_topic_name);

  ~Gripper();

  DDSPublisher *grip_action_pub_;

  void setAngleSym(int angle);

  void setAngleAsym(int front_angle, int back_angle);

  int getMaxAngle() {return MAX_ANGLE;};

private:
  std::string id_ = "N/A";
  cpp_msg::RotGripCmd_msg grip_action_cmd_{};

  // TODO
  const int MAX_ANGLE = 85;

  bool debug = false;
};