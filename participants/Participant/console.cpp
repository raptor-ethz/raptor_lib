#include "Participant.h"

void raptor::Participant::consoleDebug(std::string &log, const std::string &message) {
  log += "[" + getTime() + "|DEBUG][" + id_ + "] " + message + '\n';

  if (console_state_ == 0) {
    std::cout << "\033[1m[" << getTime() << "|DEBUG]\033[1;36m[" << id_
              << "] \033[0m" << message << std::endl;
  }
}

void raptor::Participant::consoleInformation(std::string &log, const std::string &message) {
  log += "[" + getTime() + "|INFO][" + id_ + "] " + message + '\n';

  if (console_state <= 1) {
    std::cout << "\033[1;32m[" << getTime() << "|INFO]\033[1;36m[" << id_
              << "] \033[1;32m" << message << "\033[0m" << std::endl;
  }
}

void raptor::Participant::consoleWarningTest(std::string &log, const std::string &message) {
  log += "[" + getTime() + "|WARNING][" + id_ + "] " + message + '\n';

  if (console_state <= 2) {
    std::cout << "\033[1;33m[" << getTime() << "|WARNING]\033[1;36m[" << id_
              << "] \033[1;33m" << message << "\033[0m" << std::endl;
  }
}

void raptor::Participant::consoleErrorTest(std::string &log, const std::string &message)
{
  log += "[" + getTime() + "|ERROR][" + id_ + "] " + message + '\n';

  if (console_state <= 3) {
    std::cout << "\033[1;31m[" << getTime() << "|ERROR]\033[1;36m[" << id_
              << "] \033[1;31m" << message << "\033[0m" << std::endl;
  }
}