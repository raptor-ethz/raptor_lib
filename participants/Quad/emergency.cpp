#include "Quad.h"

void Quad::emergencyLand()
{
  if (!(gripper_ == nullptr)) {
    // TODO: open gripper (add doc!!!!)
    // gripper_->set_angle_sym()
    // wait!
  }
  // terminate offboard
  pos_cmd_.header.id = "break";
  position_pub_->publish(pos_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // default land command
  px4_action_cmd_.id = "land";
  px4_action_pub_->publish(px4_action_cmd_);

  // exit this process
  std::cout << "[WARNING][Participand: " << id_
            << "] Default land now. Exiting process." << std::endl;
  exit(0);
}

// TODO hover advise
void Quad::hover()
{
  // send hover command
  pos_cmd_.header.id = "hover";
  position_pub_->publish(pos_cmd_);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  // advise
  std::string input;
  while (true) {
    std::cout << "Advise next action [e = emergency land, q = quit]: ";
    getline(std::cin, input);
    if (input.length() > 1) {
      std::cout << "Enter 1 character only!" << std::endl;
      continue;
    }
    switch (input.at(0)) {
    // emergency land
    case 'e':
      std::cout << "Sending emergency land command." << std::endl;
      emergencyLand();

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