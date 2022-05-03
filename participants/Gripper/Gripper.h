#pragma once

#include "RotGripCmd_msg.h"
#include "RotGripCmd_msgPubSubTypes.h"
#include "GripperSensor_msg.h"
#include "GripperSensor_msgPubSubTypes.h"
#include "domain_participant.h"
#include "publisher.h"
#include "subscriber.h"

// TODO
enum GripperConsoleState
{
  gripper_console_debug,
  gripper_console_info,
  gripper_console_warning,
  gripper_console_error,
  gripper_console_none
};

class Gripper
{
public:
  Gripper(const std::string &raptor_participant_id, std::string *const log,
          std::unique_ptr<DefaultParticipant> &dp,
          const std::string &pub_topic_name);

  ~Gripper();

  DDSPublisher *grip_action_pub_;
  DDSSubscriber<idl_msg::GripperSensor_msgPubSubType, cpp_msg::GripperSensor_msg> *grip_sensor_sub_;

  void setAngleSym(int angle);

  void setAngleAsym(int front_angle, int back_angle);

  void triggerGripper();
  void stopTriggerGripper();

  void updateSensor();

  int getSensorBackLeft();
  int getSensorBackRight();
  int getSensorFrontLeft();
  int getSensorFrontRight();

  int getMaxAngle() { return MAX_ANGLE; };

  // console
  void consoleDebug(const std::string &message);
  void consoleInformation(const std::string &message);
  void consoleWarning(const std::string &message);
  void consoleError(const std::string &message);

private:
  std::string id_ = "N/A";
  std::string *log_{nullptr};
  cpp_msg::RotGripCmd_msg grip_action_cmd_{};
  cpp_msg::GripperSensor_msg grip_sensor_msg_{};
  GripperConsoleState console_state_{gripper_console_debug};

  // sensor values

  // TODO
  const int MAX_ANGLE = 85;
  const std::string SENSOR_TOPIC_ = "gripper_sensor_msg";
};