#include "Quad.h"

Status Quad::getStatus() {
  // send status request
  px4_action_cmd_.action = Action_cmd::act_status;
  px4_action_pub_->publish(px4_action_cmd_);

  px4_feedback_sub_->listener->wait_for_data_for_ms(2000);

  Status result;
  // check if feedback was received
  if (px4_feedback_.feedback != FeedbackType::fb_status) {
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
  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Running preflight checks."
              << std::endl;
  }

  // initialize mocap subscriber
  if (!initializeMocapSub()) {
    // error
    std::cout << "[ERROR][Participant: " << id_
              << "] Takeoff denied: Initialization failed." << std::endl;
    return false;
  }

  for (int i = 5;; --i) {
    // check status
    Status status = getStatus();
    // check if feedback received
    if (!status.feedback) {
      // error
      std::cout << "[ERROR][Participant: " << id_
                << "] Takeoff denied: No feedback received from interface."
                << std::endl;
      return false;
    }
    // check if local position is available
    if (!status.local_position) {
      // error
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
        // error
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

  state_ = State::initialized;

  /* ARM */
  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Arming." << std::endl;
  }
  // send arm request
  px4_action_cmd_.action = Action_cmd::act_arm;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait max for 2 seconds to receive data
  px4_feedback_sub_->listener->wait_for_data_for_ms(2000);

  // check if feedback was received
  if (px4_feedback_.feedback != FeedbackType::fb_arm) {
    std::cout << "[ERROR][Participant: " << id_
              << "] Arming failed: No feedback received from interface."
              << std::endl;
    return false;
  }
  // check Result
  if (px4_feedback_.result != ResultType::res_success) {
    std::cout << "[ERROR][Participant: " << id_ << "] Arming failed: PX4 error."
              << std::endl;
    return false;
  }

  state_ = armed;

  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(1800));

  /* TAKEOFF */
  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Taking off." << std::endl;
  }

  // send takeoff request
  px4_action_cmd_.action = Action_cmd::act_takeoff;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait max for 2 seconds to receive data
  px4_feedback_sub_->listener->wait_for_data_for_ms(2000);

  // check if feedback was received
  if (px4_feedback_.feedback != FeedbackType::fb_takeoff) {
    std::cout << "[ERROR][Participant: " << id_
              << "] Takeoff failed: No feedback received from interface."
              << std::endl;
    return false;
  }
  // check Result
  if (px4_feedback_.result != ResultType::res_success) {
    std::cout << "[ERROR][Participant: " << id_
              << "] Takeoff failed: PX4 error." << std::endl;
    return false;
  }

  // wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(12000));

  // TODO : check height
  // if (pose_.position.z < TODO) {
  //   // error!
  // }

  // debug
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_
              << "] Take-off sequence completed." << std::endl;
  }

  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Starting offboard."
              << std::endl;
  }

  /* OFFBOARD */
  // send offboard request
  px4_action_cmd_.action = Action_cmd::act_offboard;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait max for 2 seconds to receive data
  // px4_feedback_sub_->listener->wait_for_data_for_ms(2000);

  // // check if feedback was received
  // if (px4_feedback_.feedback != FeedbackType::offboard) {
  //   std::cout << "[ERROR][Participant: " << id_
  //             << "] Offboard failed: No feedback received from interface."
  //             << std::endl;
  //   return false;
  // }
  // // check Result
  // if (px4_feedback_.result != ResultType::success) {
  //   std::cout << "[ERROR][Participant: " << id_
  //             << "] Offboard failed: PX4 error." << std::endl;
  //   return false;
  // }

  // wait for the drone to stabilize
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  state_ = airborne;

  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_
              << "] Takeoff complete. Ready to fly mission." << std::endl;
  }

  return true;
}

void Quad::land(Item &stand) {
  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Commence landing sequence."
              << std::endl;
  }
  // debug
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Go back to stand."
              << std::endl;
  }
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 1.0, stand.getPose().orientation.yaw,
          5000, false);

  // debug
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Descending." << std::endl;
  }
  // z + 0.5
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 0.5, stand.getPose().orientation.yaw,
          2000, false);
  // z + 0.2
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 0.2, stand.getPose().orientation.yaw,
          2000, false);
  // z + 0.0
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z, stand.getPose().orientation.yaw, 2000,
          false);

  // TODO check offset!

  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Landing." << std::endl;
  }

  // z - 0.3
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z - 0.3, stand.getPose().orientation.yaw, 2000,
          false);

  // terminate offboard
  pos_cmd_.header.description = "break";
  position_pub_->publish(pos_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // default land command
  px4_action_cmd_.action = Action_cmd::act_land;
  px4_action_pub_->publish(px4_action_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  // back up disarm command
  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Safety Disarm."
              << std::endl;
  }
  px4_action_cmd_.action = Action_cmd::act_disarm;
  px4_action_pub_->publish(px4_action_cmd_);

  state_ = initialized;

  // kill?
}
