#include "Quad.h"

void Quad::emergencyLand()
{
  if (gripper_ != nullptr)
  {
    gripper_->setAngleSym(gripper_->getMaxAngle());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // default land command
  px4_action_cmd_.action = Action_cmd::act_land;
  px4_action_pub_->publish(px4_action_cmd_);
  std::cout << "[WARNING][Participand: " << id_
            << "] Default land now." << std::endl;
}

void Quad::hover()
{
  // send hover command
  px4_action_cmd_.action = Action_cmd::act_hover; // TODO -> does it work?
  px4_action_pub_->publish(px4_action_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  // advise
  std::string input;
  while (true)
  {
    std::cout << "Advise next action [e = emergency land, q = quit]: ";
    getline(std::cin, input);
    if (input.length() > 1)
    {
      std::cout << "Enter 1 character only!" << std::endl;
      continue;
    }
    switch (input.at(0))
    {
    // emergency land
    case 'e':
      std::cout << "Sending emergency land command." << std::endl;
      emergencyLand();
      // break;
    // quit
    case 'q':
      std::cout << "Exiting programm." << std::endl;
      exit(0);

    default:
      std::cout << "Unknown command: " << input << std::endl;
      break;
    }
  }

  assert(false);
}