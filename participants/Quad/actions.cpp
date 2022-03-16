#include "Quad.h"

bool Quad::checkMocapData()
{
  long frame_number = getPose().header.timestamp;
  if (frame_number == 0 || frame_number == old_frame_number_) {
    ++missed_frames_;
  } else {
    missed_frames_ = 0;
  }
  // update old frame number
  old_frame_number_ = frame_number;
  // check error
  if (missed_frames_ > 2) {
    // Error
    std::cout << "[ERROR][Participant: " << id_
              << "] Bad motion capture data detected." << std::endl;
    return false;
  }

  return true;
}

bool Quad::takeOff()
{
  if (!checkMocapData()) {
    // Error
    std::cout << "[ERROR][Participant: " << id_ << "] Take off denied."
              << std::endl;
    return false;
  }
  // TODO preflight checks
  // switch (state_) {
  // case 0:
  //   break;

  // case 1:
  // case 2:
  //   for (int i = 0; i < 10; ++i) {
  //     std::cout << "Trying again in 3 ";
  //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //     std::cout << "2 ";
  //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //     std::cout << "1" << std::endl;
  //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //     if (checkState() == 0) {
  //       break;
  //     }
  //   }
  //   return false;

  // case 3:
  //   return false;

  // default:
  //   assert(false);
  //   break;
  // }

  /* ARM */
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Arming." << std::endl;
  }

  px4_action_cmd_.id = "arm";
  px4_action_pub_->publish(px4_action_cmd_);
  state_ = armed;

  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  /* TAKEOFF */
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Taking off." << std::endl;
  }

  px4_action_cmd_.id = "takeoff";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(12000));

  // TODO : check height?

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_
              << "] Take-off sequence completed." << std::endl;
  }
  /* DEBUG END */

  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Starting offboard."
              << std::endl;
  }
  /* INFO END */

  px4_action_cmd_.id = "offboard";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait for the drone to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Switched to offboard."
              << "Ready to fly mission." << std::endl;
  }
  /* DEBUG END */

  return true;
}

void Quad::land(Item &stand)
{
  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Commence landing sequence."
              << std::endl;
  }
  /* INFO END */
  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Go back to stand."
              << std::endl;
  }
  /* DEBUG END */

  goToPos(stand.getPose().pose.position.x, stand.getPose().pose.position.y,
          stand.getPose().pose.position.z + 1.0,
          stand.getPose().pose.orientation_euler.yaw, 5000, false);

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Descending." << std::endl;
  }
  /* DEBUG END */

  goToPos(stand.getPose().pose.position.x, stand.getPose().pose.position.y,
          stand.getPose().pose.position.z + 0.5, //.75
          stand.getPose().pose.orientation_euler.yaw, 2000, false);

  goToPos(stand.getPose().pose.position.x, stand.getPose().pose.position.y,
          stand.getPose().pose.position.z + 0.2,
          stand.getPose().pose.orientation_euler.yaw, 2000, false);

  goToPos(stand.getPose().pose.position.x, stand.getPose().pose.position.y,
          stand.getPose().pose.position.z + 0.0,
          stand.getPose().pose.orientation_euler.yaw, 2000, false);

  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Landing." << std::endl;
  }
  /* INFO END */

  goToPos(stand.getPose().pose.position.x, stand.getPose().pose.position.y,
          stand.getPose().pose.position.z - 0.3,
          stand.getPose().pose.orientation_euler.yaw, 2000, false);

  // wait for the drone to land
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  // terminate offboard
  pos_cmd_.header.id = "break";
  position_pub_->publish(pos_cmd_);

  // back up disarm command
  /* INFO */
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Safety Disarm."
              << std::endl;
  }
  /* INFO END */
  px4_action_cmd_.id = "disarm";
  px4_action_pub_->publish(px4_action_cmd_);

  state_ = initialized;

  // kill?
}
