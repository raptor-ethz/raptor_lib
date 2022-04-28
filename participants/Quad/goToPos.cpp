#include "Quad.h"

/* Non-member functions */

inline bool checkReachedPos1D(const float &actual_pos,
                              const float &reference_pos,
                              const float &threshold) {
  return std::abs(reference_pos - actual_pos) <= threshold;
}

bool checkReachedPos3D(const float &x_actual, const float &x_ref,
                       const float &x_thresh, const float &y_actual,
                       const float &y_ref, const float &y_thresh,
                       const float &z_actual, const float &z_ref,
                       const float &z_thresh) {
  bool x_reach_flag = checkReachedPos1D(x_actual, x_ref, x_thresh);
  bool y_reach_flag = checkReachedPos1D(y_actual, y_ref, y_thresh);
  bool z_reach_flag = checkReachedPos1D(z_actual, z_ref, z_thresh);

  return x_reach_flag && y_reach_flag && z_reach_flag;
}

/* Member functions */

bool Quad::sendPosCmd(const float x_ref, const float y_ref, const float z_ref, const float yaw) {
  // TODO feasibility checks
  // int X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX;

  // if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX || z < Z_MIN ||
  //     z > Z_MAX) {
  //   std::cout << "[ERROR][Participant: " << id_
  //             << "] Position Command not feasible." << std::endl;
  //   return false;
  // }

  // get absolute values
  const float x = (x_ref > 0) ? x_ref : -x_ref;
  const float y = (y_ref > 0) ? y_ref : -y_ref;
  const float z = (z_ref > 0) ? z_ref : -z_ref;

  if (x < 0.001 && y < 0.001 && z < 0.001) {
    consoleError("Position command not feasible (0).");
    return false;
  }

  pos_cmd_.position.x = x_ref;
  pos_cmd_.position.y = y_ref;
  pos_cmd_.position.z = z_ref;
  pos_cmd_.yaw_angle = yaw;
  // publish pos_cmd
  position_pub_->publish(pos_cmd_);

  return true;
}

// full config
bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const float &x_thresh,
                   const float &y_thresh, const float &z_thresh,
                   const int &delay_time, const float &max_time,
                   const bool &reached_pos_flag) {
  consoleDebug("Going to position: [" + std::to_string(x_ref) + ", " +
               std::to_string(y_ref) + ", " + std::to_string(z_ref) + ", " +
               std::to_string(yaw_ref) + "] during max " +
               std::to_string(max_time) + "ms.");

  // timestamp
  std::chrono::time_point<std::chrono::steady_clock> loop_timer;
  // resulting bool
  bool result = false;

  for (float t = 0; t < max_time; t += delay_time) {
    // get start time
    loop_timer = std::chrono::steady_clock::now();
    // check mocap TODO
    if (!checkMocapData()) {
      // state_ = State::hover;
    }
    // check external message
    // check if subscriber is connected, otherwise skip
    if (ui_sub_->listener->matched()) {
      switch (ui_cmd_.command) {
      // skip on status
      case User_cmd::ui_null:
        break;

      case User_cmd::ui_hover:
        state_ = State::hover;
        ui_cmd_.command = User_cmd::ui_null;
        break;

      case User_cmd::ui_emg_land:
        state_ = State::emg_land;
        ui_cmd_.command = User_cmd::ui_null;
        break;

      case User_cmd::ui_land:
        state_ = State::land;
        ui_cmd_.command = User_cmd::ui_null;
        break;

      default:
        break;
      }
    }

    // check if interface is matched
    while (!px4_action_pub_->listener.matched()) {
      consoleError("Connection to PX4 interface lost. Reconnecting...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // check flag
    // TODO move out to (reusable) separate function
    switch (state_) {
    case State::land:
      consoleWarning("Flag interruption: Land in stand.");
      // check if a stand is registered
      if (stand_ != nullptr) {
        // change state to airborne
        state_ = State::airborne;
        // call land
        land(*stand_);
        // exit programm
        exit(0);
      } else {
        consoleError("No stand registered. Activate hover mode.");
        state_ = State::airborne;
        hover();
      }
      break;

    case State::emg_land:
      consoleWarning("Flag interruption: Emergency land.");
      emergencyLand();
      exit(0);
      break;

    case State::hover:
      consoleWarning("Flag interruption: Hover.");
      state_ = State::airborne;
      hover();
      break;
    }

    // check if reference position has been reached
    result =
        checkReachedPos3D(pose_.position.x, x_ref, x_thresh, pose_.position.y,
                          y_ref, y_thresh, pose_.position.z, z_ref, z_thresh);

    // return if position is reached and return is desired
    if (result && reached_pos_flag) {
      if (console_state_ == 0) {
        consoleDebug("Position reached before time limit.");
      }
      return result;
    } else {
      // TODO send pos cmd
      sendPosCmd(x_ref, y_ref, z_ref, yaw_ref);
      // send new pos_cmd if position hasn't been reached
      // pos_cmd_.position.x = x_ref;
      // pos_cmd_.position.y = y_ref;
      // pos_cmd_.position.z = z_ref;
      // pos_cmd_.yaw_angle = yaw_ref;
      // // publish pos_cmd
      // position_pub_->publish(pos_cmd_);
    }

    // control frequency
    loop_timer += std::chrono::milliseconds(delay_time);
    std::this_thread::sleep_until(loop_timer);
  }

  if (result) {
    consoleDebug("Position reached after time limit.");
  } else {
    consoleDebug("Position not reached within time limit.");
  }
  return result;
}

bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const int &delay_time,
                   const float &max_time, const bool &reached_pos_flag) {
  return goToPos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_, z_thresh_,
                 delay_time, max_time, reached_pos_flag);
}
// using default threshold

bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const float &max_time,
                   const bool &reached_pos_flag) {
  return goToPos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_, z_thresh_,
                 delay_time_, max_time, reached_pos_flag);
}
// using default threshold and delay

bool Quad::goToPos(Item &target, const float &x_offset, const float &y_offset,
                   const float &z_offset, const float &yaw_ref,
                   const float &max_time, const bool &reached_pos_flag) {
  return goToPos(target.getPose().position.x + x_offset,
                 target.getPose().position.y + y_offset,
                 target.getPose().position.z + z_offset, yaw_ref, x_thresh_,
                 y_thresh_, z_thresh_, delay_time_, max_time, reached_pos_flag);
}
// go to target object with offset, default thresh and delay