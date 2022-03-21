#include "Quad.h"

void Quad::emergencyLand()
{
  if (!(gripper_ == nullptr)) {
    // TODO: open gripper (add doc!!!!)
    // gripper_->set_angle_sym()
  }
  // terminate offboard
  pos_cmd_.header.id = "break";
  position_pub_->publish(pos_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // default land command
  px4_action_cmd_.id = "land";
  px4_action_pub_->publish(px4_action_cmd_);

  // exit this process
  std::cout << "[WARNING][Participand: " << id_
            << "] Default land now. Exiting process." << std::endl;
  exit(0);
}