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

void Gripper::set_angle_sym(int angle) {
  grip_cmd.position.x = 2 * max_angle + angle;
  std::cout << "cmd: " << grip_cmd.position.x << std::endl;
  grip_pub->publish(grip_cmd);
}
void Gripper::set_front_arm(int angle) {
  grip_cmd.position.x = angle;
  std::cout << "cmd: " << grip_cmd.position.x << std::endl;
  grip_pub->publish(grip_cmd);
}
void Gripper::set_back_arm(int angle) {
  grip_cmd.position.x = max_angle + angle;
  std::cout << "cmd: " << grip_cmd.position.x << std::endl;
  grip_pub->publish(grip_cmd);
}