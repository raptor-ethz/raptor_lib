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

Status Quad::getStatus()
{
  Status result;
  // try 10 times to get a feedback (total 5s)
  for (int i = 0; i < 10; ++i) {
    // send status request 5 times
    px4_action_cmd_.id = "status";
    for (int i = 0; i < 5; ++i) {
      px4_action_pub_->publish(px4_action_cmd_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // check if feedback is received
    if (px4_info_.id == "status") {
      result.feedback = true;
      // check killed
      int killed = px4_info_.timestamp / 10000;
      assert(killed == 1 || killed == 0);
      result.killed = killed;
      // check local position
      int local_position = (px4_info_.timestamp % 10000) / 1000;
      assert(local_position == 1 || local_position == 0);
      result.local_position = local_position;
      // check battery
      int battery = px4_info_.timestamp % 1000;
      assert(battery <= 100 && battery >= 0);
      result.battery = battery;
      // return result
      return result;
    }
  }
  // no feedback received after 10 times
  result.feedback = false;
  return result;
}

bool Quad::takeOff()
{
  /* PREFLIGHT CHECKS */
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Running preflight checks."
              << std::endl;
  }

  // check state
  if (!(state_ == initialized)) {
    // Error
    std::cout << "[ERROR][Participant: " << id_
              << "] Takeoff denied: Participant not initialized." << std::endl;
    return false;
  }

  // check mocap and status
  for (int i = 5; i > 0; --i) {
    // check motion capture data
    if (!checkMocapData()) {
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Bad motion capture data." << std::endl;
      // return if it is the last try
      if (i == 1) {
        // Error
        std::cout << "[ERROR][Participant: " << id_
                  << "] Takeoff denied: Bad motion capture data." << std::endl;
        return false;
      }
      // Error
      std::cout << "Rerunning preflight checks in 3 seconds (remaining tries: "
                << i - 1 << ")." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }
    // check status
    Status status = getStatus();
    // check if feedback received
    if (!status.feedback) {
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Takeoff denied: No feedback received from interface."
                << std::endl;
      return false;
    }
    // check if local position is available
    if (!status.local_position) {
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Takeoff denied: No local position available."
                << std::endl;
      return false;
    }
    // check battery level
    if (status.battery < 30) {
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Takeoff denied: Battery too low (" << status.battery
                << "%)" << std::endl;
      return false;
    }
    // check killed
    if (status.killed) {
      // Error
      std::cout << "[ERROR][Participant: " << id_ << "] Is killed."
                << std::endl;
      // return if it is the last try
      if (i == 1) {
        // Error
        std::cout << "[ERROR][Participant: " << id_
                  << "] Takeoff denied: Participant is killed." << std::endl;
        return false;
      }
      // Error
      std::cout << "Rerunning preflight checks in 3 seconds (remaining tries: "
                << i - 1 << ")." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }
    // all checks passed -> return
    break;
  }

  /* ARM */
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Arming." << std::endl;
  }
  // send arm request
  px4_action_cmd_.id = "arm";
  for (int i = 0; i < 10; ++i) {
    px4_action_pub_->publish(px4_action_cmd_);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // try 3 times to receive a feedback
    for (int j = 0; j < 3; ++j) {
      if (px4_info_.id == "arm result") {
        // feedback received
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (px4_info_.id == "arm result") {
      // feedback received
      if (px4_info_.timestamp == 1) {
        // success
        break;
      }
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Arming failed: PX4 error." << std::endl;
      return false;
    }
    if (i == 9) {
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Arming failed: No feedback." << std::endl;
      return false;
    }
  }
  state_ = armed;
  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(1800));

  /* TAKEOFF */
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Taking off." << std::endl;
  }

  // send takeoff request
  px4_action_cmd_.id = "takeoff";
  px4_action_pub_->publish(px4_action_cmd_);
  for (int i = 0;; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (px4_info_.id == "takeoff result") {
      // feedback received
      if (px4_info_.timestamp == 1) {
        // success
        break;
      }
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Takeoff failed: PX4 error." << std::endl;
      return false;
    }
    if (i == 9) {
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Takeoff failed: No feedback received." << std::endl;
      return false;
    }
  }
  // wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(12000));
  // TODO : check height?
  // if (pose_.pose.position.z < TODO) {
  //   error!
  // }

  // DEBUG
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_
              << "] Take-off sequence completed." << std::endl;
  }

  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Starting offboard."
              << std::endl;
  }

  px4_action_cmd_.id = "offboard";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait for the drone to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  state_ = airborne;

  // DEBUG
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Switched to offboard."
              << "Ready to fly mission." << std::endl;
  }

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

  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Landing." << std::endl;
  }

  goToPos(stand.getPose().pose.position.x, stand.getPose().pose.position.y,
          stand.getPose().pose.position.z - 0.3,
          stand.getPose().pose.orientation_euler.yaw, 2000, false);

  // terminate offboard
  pos_cmd_.header.id = "break";
  position_pub_->publish(pos_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // default land command
  px4_action_cmd_.id = "land";
  px4_action_pub_->publish(px4_action_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  // back up disarm command
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Safety Disarm."
              << std::endl;
  }
  px4_action_cmd_.id = "disarm";
  px4_action_pub_->publish(px4_action_cmd_);

  state_ = initialized;

  // kill?
}
