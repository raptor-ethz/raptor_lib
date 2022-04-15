#include "Quad.h"

void Quad::emergencyLand() {
  if (gripper_ != nullptr) {
    consoleDebug("Opening gripper.");
    gripper_->setAngleSym(gripper_->getMaxAngle());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // default land command
  px4_action_cmd_.action = Action_cmd::act_land;
  px4_action_pub_->publish(px4_action_cmd_);
  consoleDebug("Default land.");
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

void Quad::hover()
{
  // stay at current position
  goToPos(pose_.position.x, pose_.position.y, pose_.position.z,
          pose_.orientation.yaw, 4000, false);

  // advise next action
  std::string input;
  while (true) {
    std::cout << "Advise next action [c = continue, e = emergency land, q = "
                 "quit]: ";
    getline(std::cin, input);

    // check input length
    if (input.length() != 1) {
      std::cout << "Enter exactly 1 character!" << std::endl;
      continue;
    }
    
    switch (input.at(0)) {
    // emergency land
    case 'e':
      consoleWarning("Sending emergency land command.");
      emergencyLand();
      exit(0);
      // break;

    case 'q':
      consoleWarning("Exiting programm.");
      exit(0);

    case 'c':
      consoleWarning("Continue mission.");
      return;

    default:
      std::cout << "Unknown command: " << input << std::endl;
      break;
    }
  }

  assert(false);
}