#include "Quad.h"

bool Quad::takeOff() {
  switch (state_)
  {
  case 0:
    std::cout << "[ERROR][Participant: " << id 
      << "] Take-off rejected: Participant uninitialized!" << std::endl;
    return false;
    break;
  
  case 3:
    std::cout << "[ERROR][Participant: " << id 
      << "] Take-off rejected: Participant already airborne!" << std::endl;
    return false;
    break;
  
  default:
    break;
  }
  // TODO check unkilled?


  /* ARM */
  /* INFO */
  if (console_state_ <= 1)
  {
    std::cout << "[INFO][Particpant: "<< id << "] Arming." << std::endl;
  }
  /* INFO END */

  px4_action_cmd_.id = "arm";
  px4_action_pub_->publish(px4_action_cmd_);

  // wait before take-off
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));


  /* TAKEOFF */
  /* INFO */
  if (console_state_ <= 1)
  {
    std::cout << "[INFO][Particpant: "<< id << "] Taking off." << std::endl;
  }
  /* INFO END */
  
  px4_action_cmd_.id = "takeoff";
  px4_action_pub_->publish(px4_action_cmd_);

  //wait during take-off sequence
  std::this_thread::sleep_for(std::chrono::milliseconds(8000));

  /* DEBUG */
  if (console_state_ == 0)
  {
    std::cout << "[INFO][Particpant: "<< id << "] Take-off sequence completed." << std::endl;
  }
  /* DEBUG END */

  return true;
}