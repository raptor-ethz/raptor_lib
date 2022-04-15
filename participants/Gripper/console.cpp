#include "Gripper.h"
#include "fshelper.h"
#include <iostream>

void Gripper::consoleDebug(const std::string &message) {
  *log_ += "[" + getTime() + "|DEBUG][" + id_ + "] " + message + '\n';

  if (console_state_ == 0) {
    std::cout << "\033[1m[" << getTime() << "|DEBUG]\033[1;36m[" << id_
              << "] \033[0m" << message << std::endl;
  }
}

void Gripper::consoleInformation(const std::string &message) {
  *log_ += "[" + getTime() + "|INFO][" + id_ + "] " + message + '\n';

  if (console_state_ <= 1) {
    std::cout << "\033[1;32m[" << getTime() << "|INFO]\033[1;36m[" << id_
              << "] \033[1;32m" << message << "\033[0m" << std::endl;
  }
}

void Gripper::consoleWarning(const std::string &message) {
  *log_ += "[" + getTime() + "|WARNING][" + id_ + "] " + message + '\n';

  if (console_state_ <= 2) {
    std::cout << "\033[1;33m[" << getTime() << "|WARNING]\033[1;36m[" << id_
              << "] \033[1;33m" << message << "\033[0m" << std::endl;
  }
}

void Gripper::consoleError(const std::string &message) {
  *log_ += "[" + getTime() + "|ERROR][" + id_ + "] " + message + '\n';

  if (console_state_ <= 3) {
    std::cout << "\033[1;31m[" << getTime() << "|ERROR]\033[1;36m[" << id_
              << "] \033[1;31m" << message << "\033[0m" << std::endl;
  }
}