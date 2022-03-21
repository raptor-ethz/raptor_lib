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

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] Go to position: [\t"
              << x_ref << ",\t" << y_ref << ",\t" << z_ref << "\t] during max "
              << max_time << "ms." << std::endl;
  }
  /* DEBUG END */

  // timestamp
  std::chrono::time_point<std::chrono::steady_clock> loop_timer;
  // resulting bool
  bool result = false;

  for (float t = 0; t < max_time; t += delay_time) {
    // get start time
    loop_timer = std::chrono::steady_clock::now();
    // check mocap
    if (!checkMocapData()) {
      state_ = hover;
    }
    // TODO: check external message
    // subscriber->listener->matched?
    // read data
    // check if not 'default'

    // check flag
    // TODO move out to (reusable) separate function
    switch (state_) {
    // land
    case 4:
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Activate Failsafe: Land." << std::endl;
      if (!(stand_ == nullptr)) {
        // change state to airborne
        state_ = airborne;
        // call land
        land(*stand_);
        // exit programm
        exit(0);
      } else {
        std::cout << "[ERROR][Participant: " << id_ << "] No stand registered!"
                  << std::endl;
        // TODO: hover and advise!
      }

      break;

    // emg_land
    case 5:
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Activate Failsafe: Emergency Land." << std::endl;
      emergencyLand();
      break;

    // hover
    case 6:
      // Error
      std::cout << "[ERROR][Participant: " << id_
                << "] Activate Failsafe: Hover." << std::endl;
      // TODO: hover;
      break;
    }

    // check if reference position has been reached
    result = checkReachedPos3D(pose_.pose.position.x, x_ref, x_thresh,
                               pose_.pose.position.y, y_ref, y_thresh,
                               pose_.pose.position.z, z_ref, z_thresh);

    if (result && reached_pos_flag) {
      /* DEBUG */
      if (console_state_ == 0) {
        std::cout << "[DEBUG][Participant: " << id_
                  << "] Position reached before time limit." << std::endl;
      }
      /* DEBUG END */

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

  /* DEBUG */
  if (console_state_ == 0) {
    std::cout << "[DEBUG][Participant: " << id_ << "] ";
    if (result) {
      std::cout << "Position reached after time limit." << std::endl;
    } else {
      std::cout << "Position was not reached within time limit." << std::endl;
    }
  }
  /* DEBUG END */

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

bool Quad::go_to_pos_min_jerk(const Vec3 &pos_ref, const Vec3 &vel_ref,
                              const Vec3 &acc_ref, const int &completion_time)
{
  // TODO: caluclate current pos, velocity and acceleration

  // evaluate current position
  position_ =
      Vec3(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z);

  // instantiate trajectory
  RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator traj(
      position_, velocity_, acceleration_, gravity_);

  // define reference states
  traj.SetGoalPosition(pos_ref);
  traj.SetGoalVelocity(vel_ref);
  traj.SetGoalAcceleration(acc_ref);

  // generate trajectory
  traj.Generate(completion_time);

  // DEBUG
  std::cout << "Go to position (minJerk): [\t" << pos_ref[0] << ",\t"
            << pos_ref[1] << ",\t" << pos_ref[2] << "\t] during "
            << completion_time << "s ." << std::endl;
  // DEBUG END

  // convert delay_time to seconds
  const float dt = float(delay_time_) / 1000.0;

  // start controlling loop
  for (double i = 0; i < completion_time; i += dt) {
    // update pos_cmd
    pos_cmd_.position.x = traj.GetPosition(i).x;
    pos_cmd_.position.y = traj.GetPosition(i).y;
    pos_cmd_.position.z = traj.GetPosition(i).z;

    // DEBUG
    std::cout << "Timestep:" << i << std::endl;
    std::cout << "Position_cmd:" << '\t' << traj.GetPosition(i).x << '\t'
              << traj.GetPosition(i).y << '\t' << traj.GetPosition(i).z
              << std::endl;
    std::cout << "Position_quad:" << '\t' << pose_.pose.position.x << '\t'
              << pose_.pose.position.y << '\t' << pose_.pose.position.z
              << std::endl;
    // DEBUG END

    // publish command
    position_pub_->publish(pos_cmd_);

    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_time_));
  }

  // check if reference position has been reached
  bool result = checkReachedPos3D(pose_.pose.position.x, pos_ref[0], x_thresh_,
                                  pose_.pose.position.y, pos_ref[1], y_thresh_,
                                  pose_.pose.position.z, pos_ref[2], z_thresh_);

  // DEBUG
  if (result) {
    std::cout << "Position reached." << std::endl;
  } else {
    std::cout << "Position wasn't reached." << std::endl;
  }
  // DEBUG END

  return result;
}