#include "Gripper.h"

Gripper::Gripper(std::string &raptor_participant_id,
                 std::unique_ptr<DefaultParticipant> &dp,
                 std::string &pub_topic_name) {
  id = raptor_participant_id;
  grip_pub = new DDSPublisher(idl_msg::QuadPositionCmdPubSubType(),
                              pub_topic_name, dp->participant());
  grip_pub->init();
};
Gripper::~Gripper() { delete grip_pub; }

void Gripper::open() {
  grip_cmd.position.x = 1;
  grip_pub->publish(grip_cmd);
}

void Gripper::close() {
  grip_cmd.position.x = 0;
  grip_pub->publish(grip_cmd);
}