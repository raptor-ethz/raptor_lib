#include "Gripper.h"

Gripper::Gripper(const std::string &raptor_participant_id,
                 std::unique_ptr<DefaultParticipant> &dp,
                 const std::string &pub_topic_name) {
  id_ = raptor_participant_id;
  grip_action_pub_ = new DDSPublisher(idl_msg::RotGripCmd_msgPubSubType(),
                                      pub_topic_name, dp->participant());
};

Gripper::~Gripper() { delete grip_action_pub_; }

void Gripper::setAngleSym(int angle) {
  // check argument feasability
  // TODO change to feedback
  assert(angle >= 0 && angle <= MAX_ANGLE);

  grip_action_cmd_.front_arm_deg = angle;
  grip_action_cmd_.back_arm_deg = angle;
  grip_action_pub_->publish(grip_action_cmd_);

  if (debug) {
    std::cout << "[DEBUG][Participant: " << id_
              << "] Set angle: [ Front: " << angle << "; Back: " << angle
              << "]." << std::endl;
  }
}

void Gripper::setAngleAsym(int front_angle, int back_angle) {
  // check argument feasability
  // TODO change to feedback
  assert(front_angle >= 0 && front_angle <= MAX_ANGLE);
  assert(back_angle >= 0 && back_angle <= MAX_ANGLE);

  grip_action_cmd_.front_arm_deg = front_angle;
  grip_action_cmd_.back_arm_deg = back_angle;
  grip_action_pub_->publish(grip_action_cmd_);

  if (debug) {
    std::cout << "[DEBUG][Participant: " << id_
              << "] Set angle: [ Front: " << front_angle
              << "; Back: " << back_angle << "]." << std::endl;
  }
}