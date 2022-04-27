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
    // return error after the n-th try
    if (i == n) {
      consoleError(
          "Initialization fialed: Failed to match interface publisher.");
      return false;
    }
    // rerun checks
    consoleWarning("Interface publisher did not match (check that position_"
                   "control_interface is running).");
    consoleWarning("Rerunning initialization in 3 seconds (remaining tries: " +
                   std::to_string(n - i) + ").");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  consoleInformation("Interface publisher matched.");
  return true;
}

bool Quad::takeOff() {
  // initialize mocap subscriber
  if (!initializeMocapSub()) {
    consoleError(
        "Takeoff denied: Initialization of motion capture subscriber failed.");
    return false;
  }

  // initialize interface publisher
  if (!initializeInterfacePub()) {
    consoleError(
        "Takeoff denied: Initialization of interface publisher failed.");
    return false;
  }

  consoleInformation("Running prefilght checks.");
  // check status for max n times (3 seconds each)
  // define max tries
  const int n = 5;
  for (int i = 1;; ++i) {
    Status status = getStatus();
    // return if no feedback was received
    if (!status.feedback) {
      consoleError(
          "Takeoff denied: No feedback from interface (check px4 connection).");
      return false;
    }
    // check if local position is available
    if (!status.local_position) {
      consoleError("Takeoff denied: No local position available (check "
                   "mocap_px4_publisher).");
      return false;
    }
    // check battery level
    if (status.battery < 30) {
      consoleError("Takeoff denied: Battery too low (" +
                   std::to_string(status.battery) + "%).");
      return false;
    }
    // check killed
    if (!status.armable) {
      // return error after the n-th try
      if (i == n) {
        consoleError("Takeoff denied: Participant is killed.");
        return false;
      }
      // rerun checks
      consoleWarning("Participant is killed (check remote).");
      consoleWarning("Rerunning preflight checks in 3 seconds (remaining tires: " +
                     std::to_string(n - i) + ").");
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }

    // all checks successfull -> break loop
    consoleInformation("Preflight checks complete (battery: " + std::to_string(status.battery) + "%).");
    break;
  }

  state_ = State::initialized;

  // TODO DEBUG
  consoleDebug("Stand height: " + std::to_string(pose_.position.z));

  // ARM
  consoleInformation("Arming.");
  // send arm request
  px4_action_cmd_.action = Action_cmd::act_arm;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait for 2 seconds max to receive data
  px4_feedback_sub_->listener->wait_for_data_for_ms(2000);

  // check if feedback was received
  if (px4_feedback_.feedback != FeedbackType::fb_arm) {
    consoleError("Takeoff denied: Arming failed (received no feedback from interface).");
    return false;
  }
  // check arming result
  if (px4_feedback_.result != ResultType::res_success) {
    consoleError("Takeoff denied: Arming failed (px4 error).");
    return false;
  }

  state_ = armed;

  // wait 2s before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // TAKEOFF
  consoleInformation("Taking off.");

  // send takeoff request
  px4_action_cmd_.action = Action_cmd::act_takeoff;
  px4_action_pub_->publish(px4_action_cmd_);
  // wait max for 2 seconds to receive data
  px4_feedback_sub_->listener->wait_for_data_for_ms(2000);

  // check if feedback was received
  if (px4_feedback_.feedback != FeedbackType::fb_takeoff) {
    consoleError("Takeoff failed: Received no feedback from interface.");
    // TODO ask for land command
    return false;
  }
  // check takeoff result
  if (px4_feedback_.result != ResultType::res_success) {
    consoleError("Takeoff failed: PX4 error.");
    // TODO ask for land command
    return false;
  }

  // wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(12000));

  // check height (and wait) TODO
  if (pose_.position.z < 0.5) {
    consoleError("Takeoff failed: Insufficient height reached.");
  }
  // wait until at least 1m height was reached
  for(int i = 1; pose_.position.z < 1; ++i) {
    consoleWarning("Insufficient height for offboard (" + std::to_string(pose_.position.z) + "). Retry in 3 seconds");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  // OFFBOARD
  consoleInformation("Starting offboard control.");
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

  consoleInformation("Takeoff sequence complete. Ready to fly mission.");
  return true;
}

void Quad::land(Item &stand) {
  consoleInformation("Commencing landing sequence.");
  consoleDebug("Going back to stand.");
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z + 1.0, stand.getPose().orientation.yaw,
          5000, false);

  consoleDebug("Descending.");
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

  // wait until offset to stand is small enough
  // TODO offsets
  while (!checkReachedPos3D(pose_.position.x, stand.getPose().position.x, 0.08,
                            pose_.position.y, stand.getPose().position.y, 0.08,
                            pose_.position.z, stand.getPose().position.z,
                            0.3)) {
    consoleWarning("Offset to stand is too big for landing.");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  consoleDebug("Landing.");
  // z - 0.3
  goToPos(stand.getPose().position.x, stand.getPose().position.y,
          stand.getPose().position.z - 0.3, stand.getPose().orientation.yaw,
          2000, false);

  // default land command
  px4_action_cmd_.action = Action_cmd::act_land;
  px4_action_pub_->publish(px4_action_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));

  // back up disarm command
  consoleDebug("Safety disarm.");
  px4_action_cmd_.action = Action_cmd::act_disarm;
  px4_action_pub_->publish(px4_action_cmd_);

  state_ = initialized;
}
