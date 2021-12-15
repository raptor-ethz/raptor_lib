#include "Gripper.h"

Gripper::Gripper(const std::string &raptor_participant_id,
                 std::unique_ptr<DefaultParticipant> &dp,
                 const std::string &pub_topic_name) {
  id = raptor_participant_id;

  grip_pub = new DDSPublisher(idl_msg::QuadPositionCmdPubSubType(),
                              pub_topic_name, dp->participant());

  grip_pub->init();
};

Gripper::~Gripper() { delete grip_pub; }

void Gripper::set_angle(int angle) {
  grip_cmd.position.x = angle;
  grip_pub->publish(grip_cmd);
}