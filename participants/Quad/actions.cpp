#include "Quad.h"

bool Quad::takeOff() {
  // TODO check unkilled?


  /* ARM */
  if (console_state_ <= 1)
  {
    std::cout << "[INFO][Particpant: "<< id << "] Arming." << std::endl;
  }
  px4_action_cmd_.id = "arm";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));


  /* TAKEOFF */
  if (console_state_ <= 1)
  {
    std::cout << "[INFO][Particpant: "<< id << "] Taking off." << std::endl;
  }
  px4_action_cmd_.id = "takeoff";
  px4_action_pub_->publish(px4_action_cmd_);

  //wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(8000));

  return false;
}