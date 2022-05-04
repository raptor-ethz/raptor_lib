#include "Gripper.h"

Gripper::Gripper(const std::string &raptor_participant_id, std::string *const log,
                 std::unique_ptr<DefaultParticipant> &dp,
                 const std::string &pub_topic_name, GripperType type)
{
  id_ = raptor_participant_id;
  log_ = log;
  grip_action_pub_ = new DDSPublisher(idl_msg::GripCmd_msgPubSubType(),
                                      pub_topic_name, dp->participant());
  grip_sensor_sub_ =
      new DDSSubscriber(idl_msg::GripperSensor_msgPubSubType(),
                        &grip_sensor_msg_, SENSOR_TOPIC_, dp->participant());
  grip_action_cmd_.gripper = type;
};

Gripper::~Gripper() { delete grip_action_pub_; }

void Gripper::setAngleSym(int angle)
{
  // check argument feasability
  // TODO change to feedback
  if (angle <= 0 || angle >= MAX_ANGLE)
  {
    consoleError("Requested angle is invalid (requested " +
                 std::to_string(angle) + ", must be between [0, " +
                 std::to_string(MAX_ANGLE) + "]).");
    return;
  }

  grip_action_cmd_.servo_1_deg = angle;
  grip_action_cmd_.servo_2_deg = angle;
  grip_action_cmd_.trigger_gripper = false;
  grip_action_pub_->publish(grip_action_cmd_);

  consoleDebug("Setting symmetric angle: [" + std::to_string(angle) + "]");
}

void Gripper::setAngleAsym(int front_angle, int back_angle)
{
  // check argument feasability
  // TODO change to feedback
  if (front_angle <= 0 || front_angle >= MAX_ANGLE)
  {
    consoleError("Requested front angle is invalid (requested " +
                 std::to_string(front_angle) + ", must be between [0, " +
                 std::to_string(MAX_ANGLE) + "]).");
    return;
  }
  if (back_angle <= 0 || back_angle >= MAX_ANGLE)
  {
    consoleError("Requested back angle is invalid (requested " +
                 std::to_string(back_angle) + ", must be between [0, " +
                 std::to_string(MAX_ANGLE) + "]).");
    return;
  }

  grip_action_cmd_.servo_1_deg = front_angle;
  grip_action_cmd_.servo_2_deg = back_angle;
  grip_action_cmd_.trigger_gripper = false;
  grip_action_cmd_.request_sensor = false;
  grip_action_pub_->publish(grip_action_cmd_);

  consoleDebug("Setting asymmetric angle: [" + std::to_string(front_angle) + ", " + std::to_string(back_angle) + "]");
}

void Gripper::triggerGripper()
{
  consoleDebug("Triggering gripper.");
  grip_action_cmd_.trigger_gripper = true;
  grip_action_pub_->publish(grip_action_cmd_);
}

void Gripper::stopTriggerGripper()
{
  consoleDebug("Stop Triggering gripper.");
  grip_action_cmd_.trigger_gripper = false;
  grip_action_pub_->publish(grip_action_cmd_);
}

void Gripper::updateSensor()
{
  // just send a command to trigger the "wait_for_data()" in the gripper
  // interface
  grip_action_cmd_.request_sensor = true;
  grip_action_pub_->publish(grip_action_cmd_);
}

// sensor value getter functions
int Gripper::getSensorBackLeft() { return grip_sensor_msg_.force_back_left; }
int Gripper::getSensorBackRight() { return grip_sensor_msg_.force_back_right; }
int Gripper::getSensorFrontLeft() { return grip_sensor_msg_.force_front_left; }
int Gripper::getSensorFrontRight()
{
  return grip_sensor_msg_.force_front_right;
}