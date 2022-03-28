#include "Quad.h"

/* Non-member functions */

inline bool checkReachedPos1D(const float &actual_pos,
                              const float &reference_pos,
                              const float &threshold)
{
  return std::abs(reference_pos - actual_pos) <= threshold;
}

inline bool checkReachedPos3D(const float &x_actual, const float &x_ref,
                              const float &x_thresh, const float &y_actual,
                              const float &y_ref, const float &y_thresh,
                              const float &z_actual, const float &z_ref,
                              const float &z_thresh)
{
  bool x_reach_flag = checkReachedPos1D(x_actual, x_ref, x_thresh);
  bool y_reach_flag = checkReachedPos1D(y_actual, y_ref, y_thresh);
  bool z_reach_flag = checkReachedPos1D(z_actual, z_ref, z_thresh);

  return x_reach_flag && y_reach_flag && z_reach_flag;
}

/* Member functions */

// full config
bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const float &x_thresh,
                   const float &y_thresh, const float &z_thresh,
                   const int &delay_time, const float &max_time,
                   const bool &reached_pos_flag)
{

  // DEBUG
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Go to position: [\t"
              << x_ref << ",\t" << y_ref << ",\t" << z_ref << "\t] during max "
              << max_time << "ms." << std::endl;
  }

  // timestamp
  std::chrono::time_point<std::chrono::steady_clock> loop_timer;
  // resulting bool
  bool result = false;

  for (float t = 0; t < max_time; t += delay_time) {
    // get start time
    loop_timer = std::chrono::steady_clock::now();
    // check mocap
    if (!checkMocapData()) {
      state_ = State::hover;
    }
    // check external message
    // check if subscriber is connected, otherwise skip
    if (ui_sub_->listener->matched()) {
      switch (ui_cmd_.command)
      {
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

    // check flag
    // TODO move out to (reusable) separate function
    switch (state_) {
    case State::land:
      // error
      std::cout << "[ERROR][Participant: " << id_
                << "] Activate Failsafe: Land." << std::endl;
      // check if a stand is registered
      if (!(stand_ == nullptr)) {
        // change state to airborne
        state_ = State::airborne;
        // call land
        land(*stand_);
        // exit programm
        exit(0);
      } else {
        std::cout << "[ERROR][Participant: " << id_
                  << "] No stand registered! Activate hover mode." << std::endl;
        hover();
      }

      break;

    case State::emg_land:
      // error
      std::cout << "[ERROR][Participant: " << id_
                << "] Activate Failsafe: Emergency Land." << std::endl;
      emergencyLand();
      break;

    case State::hover:
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Activate Failsafe: Hover." << std::endl;
      hover();
      break;
    }

    // check if reference position has been reached
    result = checkReachedPos3D(pose_.position.x, x_ref, x_thresh,
                               pose_.position.y, y_ref, y_thresh,
                               pose_.position.z, z_ref, z_thresh);

    if (result && reached_pos_flag) {
      // DEBUG
      if (console_state_ == 0) {
        std::cout << "[DEBUG][Participant: " << id_
                  << "] Position reached before time limit." << std::endl;
      }

      // return from the function direclty
      return result;
    } else {
      // send new pos_cmd if position hasn't been reached
      pos_cmd_.position.x = x_ref;
      pos_cmd_.position.y = y_ref;
      pos_cmd_.position.z = z_ref;
      pos_cmd_.yaw_angle = yaw_ref;
      // publish pos_cmd
      position_pub_->publish(pos_cmd_);
    }

    // control frequency
    loop_timer += std::chrono::milliseconds(delay_time);
    std::this_thread::sleep_until(loop_timer);
  }

  // DEBUG
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] ";
    if (result) {
      std::cout << "Position reached after time limit." << std::endl;
    } else {
      std::cout << "Position was not reached within time limit." << std::endl;
    }
  }

  return result;
}

// using default threshold
bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const int &delay_time,
                   const float &max_time, const bool &reached_pos_flag)
{
  return goToPos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_, z_thresh_,
                 delay_time, max_time, reached_pos_flag);
}

// using default threshold and delay
bool Quad::goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
                   const float &yaw_ref, const float &max_time,
                   const bool &reached_pos_flag)
{
  return goToPos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_, z_thresh_,
                 delay_time_, max_time, reached_pos_flag);
}
