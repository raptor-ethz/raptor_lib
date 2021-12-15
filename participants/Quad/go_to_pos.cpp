#include <chrono>

#include "Quad.h"

inline bool check_reached_pos_1d(const float &actual_pos,
                                 const float &reference_pos,
                                 const float &threshold) {
  return std::abs(reference_pos - actual_pos) <= threshold;
}

inline bool check_reached_pos_3d(const float &x_actual, const float &x_ref,
                                 const float &x_thresh, const float &y_actual,
                                 const float &y_ref, const float &y_thresh,
                                 const float &z_actual, const float &z_ref,
                                 const float &z_thresh) {
  bool x_reach_flag = check_reached_pos_1d(x_actual, x_ref, x_thresh);
  bool y_reach_flag = check_reached_pos_1d(y_actual, y_ref, y_thresh);
  bool z_reach_flag = check_reached_pos_1d(z_actual, z_ref, z_thresh);

  return x_reach_flag && y_reach_flag && z_reach_flag;
}

bool Quad::go_to_pos(const float &x_ref, const float &y_ref, const float &z_ref,
                     const float &yaw_ref, const float &x_thresh,
                     const float &y_thresh, const float &z_thresh,
                     const int &delay_time, const float &max_time,
                     const bool &reached_pos_flag) {
  // DEBUG
  std::cout << "Go to position (standard): [\t" << x_ref << ",\t" << y_ref
            << ",\t" << z_ref << "\t] during max " << max_time << "ms ."
            << std::endl;
  // DEBUG END

  // resulting bool
  bool result = false;

  for (float timer = 0; timer < max_time; timer += delay_time) {
    // check if reference position has been reached
    result = check_reached_pos_3d(pose_.pose.position.x, x_ref, x_thresh,
                                  pose_.pose.position.y, y_ref, y_thresh,
                                  pose_.pose.position.z, z_ref, z_thresh);

    if (result && reached_pos_flag) {
      // DEBUG
      std::cout << "Position reached (return)." << std::endl;
      // DEBUG END

      // return from the function direclty
      return result;
    } else {
      // send new pos_cmd if position hasn't been reached
      pos_cmd.position.x = x_ref;
      pos_cmd.position.y = y_ref;
      pos_cmd.position.z = z_ref;
      pos_cmd.yaw_angle = yaw_ref;

      // publich pos_cmd
      position_pub->publish(pos_cmd);
    }

    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
  }

  // DEBUG
  if (result) {
    std::cout << "Position reached after time limit." << std::endl;
  } else {
    std::cout << "Position wasn't reached within time limit." << std::endl;
  }
  // DEBUG END

  return result;
}

bool Quad::go_to_pos(const float &x_ref, const float &y_ref, const float &z_ref,
                     const float &yaw_ref, const int &delay_time,
                     const float &max_time, const bool &reached_pos_flag) {
  return go_to_pos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_,
                   z_thresh_, delay_time, max_time, reached_pos_flag);
}

bool Quad::go_to_pos(const float &x_ref, const float &y_ref, const float &z_ref,
                     const float &yaw_ref, const float &max_time,
                     const bool &reached_pos_flag) {
  return go_to_pos(x_ref, y_ref, z_ref, yaw_ref, x_thresh_, y_thresh_,
                   z_thresh_, delay_time_, max_time, reached_pos_flag);
}

bool Quad::go_to_pos_min_jerk(const Vec3 &pos_ref, const Vec3 &vel_ref,
                              const Vec3 &acc_ref, const int &completion_time) {
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
    pos_cmd.position.x = traj.GetPosition(i).x;
    pos_cmd.position.y = traj.GetPosition(i).y;
    pos_cmd.position.z = traj.GetPosition(i).z;

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
    position_pub->publish(pos_cmd);

    // delay
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_time_));
  }

  // check if reference position has been reached
  bool result = check_reached_pos_3d(
      pose_.pose.position.x, pos_ref[0], x_thresh_, pose_.pose.position.y,
      pos_ref[1], y_thresh_, pose_.pose.position.z, pos_ref[2], z_thresh_);

  // DEBUG
  if (result) {
    std::cout << "Position reached." << std::endl;
  } else {
    std::cout << "Position wasn't reached." << std::endl;
  }
  // DEBUG END

  return result;
}

void Quad::land(Item stand) {
  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 1.5, 5000, false);

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 0.75, 5000, false);

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 0.2, 3000, false);

  go_to_pos(stand.get_pose().pose.position.x, stand.get_pose().pose.position.y,
            stand.get_pose().pose.position.z + 0.0, 3000, false);
}