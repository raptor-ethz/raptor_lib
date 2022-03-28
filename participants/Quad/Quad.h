#pragma once

#include "Gripper.h"
#include "Item.h"
#include "Participant.h"

#include "QuadPosCmd_msg.h"
#include "QuadPosCmd_msgPubSubTypes.h"
#include "QuadFeedback_msg.h"
#include "QuadFeedback_msgPubSubTypes.h"
#include "QuadAction_msg.h"
#include "QuadAction_msgPubSubTypes.h"
#include "UserCmd_msg.h"
#include "UserCmd_msgPubSubTypes.h"

#include "RapidTrajectoryGenerator.h"
#include "Vec3.h"

#include <chrono>

/* Non-member variables */

enum State {
  uninitialized,
  initialized,
  armed,
  airborne,
  land,
  emg_land,
  hover
};

enum ConsoleState { debug, info, warning, error };

struct Status {
  bool feedback{false};
  bool armable{false};
  bool local_position{false};
  int battery{0};
};

class Quad : public raptor::Participant {
public:
  Quad(const std::string &raptor_participant_id,
       std::unique_ptr<DefaultParticipant> &dp,
       const std::string &sub_topic_name, const std::string &pub_topic_name,
       Gripper *const gripper, Item *const stand);
  ~Quad();

  DDSPublisher *position_pub_;
  DDSPublisher *px4_action_pub_;
  DDSSubscriber<idl_msg::QuadFeedback_msgPubSubType, cpp_msg::QuadFeedback_msg>
      *px4_feedback_sub_;
  DDSSubscriber<idl_msg::UserCmd_msgPubSubType, cpp_msg::UserCmd_msg> *ui_sub_;
  

  Status getStatus();

  /**
   * @brief Comand the drone to track a position (full configuration).
   *
   * Command the drone to track a position using continuously sent position
   * commands at a specified frequency. Returns latest after the time limit or,
   * if the reached_pos_flag is set to true, once the position has been reached
   * within the specified threshold.
   * @param[in] x_ref [m] Reference position - x coordinate
   * @param[in] y_ref [m] Reference position - y coordinate
   * @param[in] z_ref [m] Reference position - z coordinate
   * @param[in] x_thresh [m] Maximal accepted deviation from x_ref
   * @param[in] y_thresh [m] Maximal accepted deviation from y_ref
   * @param[in] z_thresh [m] Maximal accepted deviation from z_ref
   * @param[in] delay_time [ms] Delay between commands
   * @param[in] max_time [ms] Time after which the function latest ends
   * @param[in] reached_pos_flag Flag if the function should end once
   * the position was reached. If false, the function always waits for
   * max_time before ending
   * @returns True if the position has been reached, false otherwise.
   */
  bool goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
               const float &yaw_ref, const float &x_thresh,
               const float &y_thresh, const float &z_thresh,
               const int &delay_time, const float &max_time,
               const bool &reached_pos_flag);

  /**
   * @brief Comand the drone to track a position (no threshold).
   *
   * Command the drone to track a position using continuously sent position
   * commands at a specified frequency. Returns latest after the time limit or,
   * if the reached_pos_flag is set to true, once the position has been reached
   * within a specified threshold.
   * @param[in] x_ref [m] Reference position - x coordinate
   * @param[in] y_ref [m] Reference position - y coordinate
   * @param[in] z_ref [m] Reference position - z coordinate
   * @param[in] delay_time [ms] Delay between commands
   * @param[in] max_time [ms] Time after which the function latest ends
   * @param[in] reached_pos_flag Flag if the function should end once
   * the position was reached. If false, the function always waits for
   * max_time before ending
   * @returns True if the position has been reached, false otherwise.
   */
  bool goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
               const float &yaw_ref, const int &delay_time,
               const float &max_time, const bool &reached_pos_flag);

  /**
   * @brief Comand the drone to track a position (no threshold, delay_time).
   *
   * Command the drone to track a position using continuously sent position
   * commands at a specified frequency. Returns latest after the time limit or,
   * if the reached_pos_flag is set to true, once the position has been reached
   * within a specified threshold.
   * @param[in] x_ref [m] Reference position - x coordinate
   * @param[in] y_ref [m] Reference position - y coordinate
   * @param[in] z_ref [m] Reference position - z coordinate
   * @param[in] max_time [ms] Time after which the function latest ends
   * @param[in] reached_pos_flag Flag if the function should end once
   * the position was reached. If false, the function always waits for
   * max_time before ending
   * @returns True if the position has been reached, false otherwise.
   */
  bool goToPos(const float &x_ref, const float &y_ref, const float &z_ref,
               const float &yaw_ref, const float &max_time,
               const bool &reached_pos_flag);

  /**
   * Compute a trajectory between the current state of the drone and the
   * specified reference state during the duration of completion_time.
   * Then, send position commands to the drone to follow the trajectory
   * using the previously defined delay_time in this instance.
   * @param[in] pos_ref : [m] Reference position
   * @param[in] vel_ref : [m] Reference velocity
   * @param[in] acc_ref : [m] Reference acceleration
   * @param[in] completion_time [s] Reference time for the drone to fly
   * the trajectory
   * @returns If the position has been reached when the function ends
   *
   **/
  bool go_to_pos_min_jerk(const Vec3 &pos_ref, const Vec3 &vel_ref,
                          const Vec3 &acc_ref, const int &completion_time);

  /**
   * @brief Arms the quad, performs the take-off and switches to offboard.
   *
   * In the first step, a proper state of the quad is asserted. If the state
   * is accepted, the quad ist armed. After sleeping for 2 seconds,
   * a take off command is sent to the drone and the state is set to ariborne.
   * The thread then waits for 8 seconds for the maneuver to complete. Then
   * it switches to offboard control mode and sleeps for additional 2 seconds
   * for the drone to stabilize before it returns.
   *
   * @return true: If the take off has been successful.
   * @return false: If an error occured.
   */
  bool takeOff();

  /**
   * @brief Commands the drone to fly back to the stand, descend and land.
   *
   * @param stand The instance of the stand to land in.
   */
  void land(Item &stand);

  /**
   * @brief Commands the drone to land immediately at its current position
   * using the defualt mavsdk land action. If a gripper is attached, it fully
   * opens the gripper first. After the command is sent, the programm
   * terminates.
   *
   */
  void emergencyLand();

  /**
   * @brief Commands the drone to hover at its current position using the
   * default mavsdk command?
   *
   * Commands the drone to hover at its current position using the default
   * mavsdk command? Then it awaits further instructions from the user
   * (emergency land or quit). Finally, the programm will terminate.
   * instructions.
   *
   */
  void hover();

  void swoop(Item &target, Gripper &gripper, float length, float dx, float dy,
             float dz, float h0, int time, int grip_angle);
  void quickSwoop(Item &target, Gripper &gripper, float length, float dx,
                  float dy, float dz, float h0, int time, int grip_angle);
  void release(Item &target, Gripper &gripper, float length, float h0,
               int time);
  void quickRelease(Item &target, Gripper &gripper, float length, float h0,
                    int time);

  void setDefaultThreshold(const float x_thresh, const float y_thresh,
                           const float z_thresh) {
    x_thresh_ = x_thresh;
    y_thresh_ = y_thresh;
    z_thresh_ = z_thresh;
  }

  void set_velocity(const Vec3 &velocity) { velocity_ = velocity; }

  // temporary (until initialization is ready)
  void setState(const State new_state) { state_ = new_state; }

  /**
   * @brief Get the state of the drone.
   *
   * @return int:
   * 0, uninitialized
   * 1, initialized
   * 2, armed
   * 3, airborne
   * 4, land
   * 5, emg_land
   * 6, hover
   */
  int getState() { return state_; }

private:
  Gripper *gripper_;
  Item *stand_;
  ConsoleState console_state_ = debug;
  State state_ = uninitialized;

  // messages
  cpp_msg::QuadPosCmd_msg pos_cmd_{};
  cpp_msg::QuadAction_msg px4_action_cmd_{};
  cpp_msg::QuadFeedback_msg px4_feedback_{};
  cpp_msg::UserCmd_msg ui_cmd_{};

  // goToPos stuff
  float x_thresh_{0.2};
  float y_thresh_{0.2};
  float z_thresh_{0.2};

  int delay_time_{20};

  // MM stuff
  Vec3 position_{0, 0, 0};
  Vec3 velocity_{0, 0, 0};
  Vec3 acceleration_{0, 0, 0};

  const Vec3 gravity_{0, 0, -9.81};
};

/* Non-member functions */

/**
 * @brief Checks if the actual 1D position lies within a specified thershold of
 * the reference position.
 *
 * @param actual_pos actual position
 * @param reference_pos reference position
 * @param threshold threshold
 */
inline bool checkReachedPos1D(const float &actual_pos,
                              const float &reference_pos,
                              const float &threshold);

/**
 * @brief Checks if the actual 3D position lies within a specified thershold of
 * the reference position.
 *
 * @param x_actual
 * @param x_ref
 * @param x_thresh
 * @param y_actual
 * @param y_ref
 * @param y_thresh
 * @param z_actual
 * @param z_ref
 * @param z_thresh
 */
inline bool checkReachedPos3D(const float &x_actual, const float &x_ref,
                              const float &x_thresh, const float &y_actual,
                              const float &y_ref, const float &y_thresh,
                              const float &z_actual, const float &z_ref,
                              const float &z_thresh);