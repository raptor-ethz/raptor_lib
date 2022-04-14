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

bool Quad::initializeInterfacePub() {
  consoleInformation("Initializing interface publisher");
  // define max tries
  const int n = 5;

  // check if publisher matched for max n times (3 seconds each)
  // exit loop if both matched
  for (int i = 1; !px4_action_pub_->listener.matched() ||
                  !position_pub_->listener.matched();
       ++i) {
    if (i == n) {
      // error if subscriber didn't match after n tries
      consoleError("Failed to match interface publisher.");
      return false;
    }
    consoleWarning("Interface publisher did not match. Check that position "
                   "control interface is running.");
    consoleWarning("Rerunning initialization in 3 seconds (remaining tries: "
                   + std::to_string(n - i) + ").");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  consoleInformation("Interface publisher matched.");
  return true;
}

bool Quad::takeOff() {
  /* PREFLIGHT CHECKS */
  consoleInformation("Running prefilght checks.");

  // initialize mocap subscriber
  if (!initializeMocapSub()) {
    consoleError("Takeoff denied: Motion Capture Subscriber Initialization failed.");
    return false;
  }

  // initialize interface publisher
  if (!initializeInterfacePub()) {
    // error
    std::cout << "[ERROR][Participant: " << id_
              << "] Takeoff denied: Interface-Publisher initialization failed."
              << std::endl;
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
      // return if it is the last try
      if (i == 1) {
        // error
        std::cout << "[ERROR][Participant: " << id_
                  << "] Takeoff denied: Participant is killed." << std::endl;
        return false;
      }
      // warning TODO
      std::cout << "[WARNING][Participant: " << id_
                << "] Is killed (check remote). Rerunning preflight checks in "
                   "3 seconds "
                   "(remaining tries: "
                << i - 1 << ")." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }
    // all checks passed -> return
    // info
    if (console_state_ <= 1) {
      std::cout << "[INFO][Participant: " << id_
                << "] Preflight checks complete (battery: " << status.battery
                << "%)." << std::endl;
    }
    break;
  }

  state_ = State::initialized;

  // TODO DEBUG
  std::cout << "Stand height: " << pose_.position.z << std::endl;

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
  // TODO offsets
  while (!checkReachedPos3D(pose_.position.x, stand.getPose().position.x, 0.1,
                            pose_.position.y, stand.getPose().position.y, 0.1,
                            pose_.position.z, stand.getPose().position.z,
                            0.3)) {
    std::cout << "[WARNING][Participant: " << id_ << "] Offset too big."
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // info
  if (console_state_ <= 1) {
    std::cout << "[INFO][Participant: " << id_ << "] Landing." << std::endl;
  }

  // z - 0.3
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z - 0.3, stand.getPose().orientation.yaw,
          2000, false);

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
