#include "Quad.h"

bool Quad::checkMocapData() {
  long frame_number = getPose().header.timestamp;
  if (frame_number == 0 || frame_number == old_frame_number_) {
    ++missed_frames_;
  } else {
    missed_frames_ = 0;
  }
  // update old frame number
  old_frame_number_ = frame_number;
  // check for 3 consecutive missed frames
  if (missed_frames_ > 2) {
    // error
    std::cout << "[ERROR][Participant: " << id_
              << "] Bad motion capture data detected." << std::endl;
    return false;
  }

  return true;
}

Status Quad::getStatus() {
  // send status request
  px4_action_cmd_.action = Action_cmd::status;
  px4_action_pub_->publish(px4_action_cmd_);

  px4_status_sub_->listener->wait_for_data_for_ms(2000);

  Status result;
  // check if feedback was received
  if (px4_feedback_.feedback != FeedbackType::status) {
    return result;
  }
  result.feedback = true;
  result.armable = px4_feedback_.status.armable;
  result.local_position = px4_feedback_.status.local_position_ok;
  result.battery = px4_feedback_.status.battery;
  return result;
}

bool Quad::takeOff() {
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

  // check mocap and status (for max. 5 tries)
  for (int i = 5;; --i) {
    // check motion capture data
    for (int j = 0; j < 2; ++j) {
      checkMocapData();
    }
    if (!checkMocapData()) {
      // error
      std::cout << "[ERROR][Participant: " << id_
                << "] Bad motion capture data." << std::endl;
      // return if it is the last try
      if (i == 1) {
        // error
        std::cout << "[ERROR][Participant: " << id_
                  << "] Takeoff denied: Bad motion capture data." << std::endl;
        return false;
      }
      // rerun checks
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
    if (!status.armable) {
      // error
      std::cout << "[ERROR][Participant: " << id_ << "] Is killed."
                << std::endl;
      // return if it is the last try
      if (i == 1) {
        // Error
        std::cout << "[ERROR][Participant: " << id_
                  << "] Takeoff denied: Participant is killed." << std::endl;
        return false;
      }
      // rerun checks
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
  px4_action_cmd_.action = Action_cmd::arm;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait max for 2 seconds to receive data
  px4_status_sub_->listener->wait_for_data_for_ms(2000);

  // TODO new message
  // check feedback
  // if (px4_feedback_.feedback != arm_result) {
  //   // Error
  //   std::cout << "[ERROR][Participant: " << id_
  //             << "] Arming failed: No feedback." << std::endl;
  //   return false;
  // }

  // check Result

  // Error
  std::cout << "[ERROR][Participant: " << id_ << "] Takeoff failed: PX4 error."
            << std::endl;
  return false;

  state_ = armed;

  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(1800));

  /* TAKEOFF */
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Taking off." << std::endl;
  }

  // send takeoff request
  px4_action_cmd_.action = Action_cmd::takeoff;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait max for 2 seconds to receive data
  px4_status_sub_->listener->wait_for_data_for_ms(2000);

  // TODO
  // check feedback
  // check result

  for (int i = 0;; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (px4_info_.id == "takeoff result") {
      // feedback received
      if (px4_info_.timestamp == 1) {
        // success
        break;
      }
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

void Quad::land(Item &stand) {
  // INFO
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Commence landing sequence."
              << std::endl;
  }
  // DEBUG
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Go back to stand."
              << std::endl;
  }
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 1.0,
          stand.getPose().orientation.yaw, 5000, false);

  // DEBUG
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Descending." << std::endl;
  }
  // z + 0.5
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 0.5,
          stand.getPose().orientation.yaw, 5000, false);
  // z + 0.2
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 0.2,
          stand.getPose().orientation.yaw, 5000, false);
  // z + 0.0
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z,
          stand.getPose().orientation.yaw, 5000, false);

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
