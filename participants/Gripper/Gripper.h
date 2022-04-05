#pragma once

#include "RotGripCmd_msg.h"
#include "RotGripCmd_msgPubSubTypes.h"
#include "GripperSensor_msg.h"
#include "GripperSensor_msgPubSubTypes.h"
#include "domain_participant.h"
#include "publisher.h"
#include "subscriber.h"

class Gripper
{
public:
  Gripper(const std::string &raptor_participant_id,
          std::unique_ptr<DefaultParticipant> &dp,
          const std::string &pub_topic_name);

  ~Gripper();

  DDSPublisher *grip_action_pub_;
  DDSSubscriber<idl_msg::GripperSensor_msgPubSubType, cpp_msg::GripperSensor_msg> *grip_sensor_sub_;

  void setAngleSym(int angle);

  void setAngleAsym(int front_angle, int back_angle);

  void triggerGripper();

  void updateSensor();

  std::vector<int> getSensorVal();

  int getMaxAngle() { return MAX_ANGLE; };

private:
  std::string id_ = "N/A";
  cpp_msg::RotGripCmd_msg grip_action_cmd_{};
  cpp_msg::GripperSensor_msg grip_sensor_msg_{};

  // sensor values

  // TODO
  const int MAX_ANGLE = 85;
  const std::string SENSOR_TOPIC_ = "gripper_sensor_msg";
  bool debug = false;
};