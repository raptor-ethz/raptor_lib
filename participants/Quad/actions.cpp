#include "Quad.h"

bool Quad::takeOff()
{
  switch (state_) {
  case 0:
    std::cout << "[ERROR][Participant: " << id
              << "] Take-off rejected: Participant uninitialized!" << std::endl;
    return false;
    break;

  case 3:
    std::cout << "[ERROR][Participant: " << id
              << "] Take-off rejected: Participant already airborne!"
              << std::endl;
    return false;
    break;

  default:
    break;
  }
  // TODO check unkilled?

  /* ARM */
  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Particpant: " << id << "] Arming." << std::endl;
  }
  /* INFO END */

  px4_action_cmd_.id = "arm";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  /* TAKEOFF */
  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Particpant: " << id << "] Taking off." << std::endl;
  }
  /* INFO END */

  px4_action_cmd_.id = "takeoff";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(8000));

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Particpant: " << id
              << "] Take-off sequence completed." << std::endl;
  }
  /* DEBUG END */

  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Particpant: " << id << "] Starting offboard."
              << std::endl;
  }
  /* INFO END */

  px4_action_cmd_.id = "offboard";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait for the drone to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Particpant: " << id << "] Switched to offboard."
              << "Ready to fly mission." << std::endl;
  }
  /* DEBUG END */

  return true;
}

void Quad::land(Item &stand)
{
  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Particpant: " << id << "] Commence landing sequence."
              << std::endl;
  }
  /* INFO END */
  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Particpant: " << id << "] Go back to stand."
              << std::endl;
  }
  /* DEBUG END */

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 1.0,
            stand.get_pose().pose.orientation_euler.yaw, 5000, false);

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Particpant: " << id << "] Descending." << std::endl;
  }
  /* DEBUG END */

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 0.75,
            stand.get_pose().pose.orientation_euler.yaw, 2000, false);

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 0.2,
            stand.get_pose().pose.orientation_euler.yaw, 2000, false);

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 0.0,
            stand.get_pose().pose.orientation_euler.yaw, 2000, false);

  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Particpant: " << id << "] Landing." << std::endl;
  }
  /* INFO END */
  px4_action_cmd_.id = "land";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait for the drone to land
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  // back up disarm command
  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Particpant: " << id << "] Safety Disarm." << std::endl;
  }
  /* INFO END */
  px4_action_cmd_.id = "disarm";
  px4_action_pub_->publish(px4_action_cmd_);

  // kill?
}