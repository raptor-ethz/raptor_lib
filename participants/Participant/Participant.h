#pragma once

#include "Mocap_msg.h"
#include "Mocap_msgPubSubTypes.h"

#include "domain_participant.h"
#include "publisher.h"
#include "subscriber.h"

#include <atomic>

enum ConsoleState {
  console_debug,
  console_info,
  console_warning,
  console_error,
  console_none
};

namespace raptor {

class Participant {
public:
  DDSSubscriber<idl_msg::Mocap_msgPubSubType, cpp_msg::Mocap_msg> *mocap_sub_;

  /**
   * @brief Checks if the subscriber has matched anything and
   * if the received data is non-zero.
   *
   * Checks for a maximum of 2 seconds if the subscriber has matched anything.
   * If something was matched it attempts to receive 10 new datapoints (for a
   * maximum of 500ms each). The data quality is then checked using
   * checkMocapData (max 5 times to receive good data).
   *
   * @see checkMocapData
   *
   * @returns If subscriber is matched and received good data.
   **/
  bool initializeMocapSub();

  /**
   * @brief Checks the current quality of the received mocap data.
   *
   * Checks if the the current frame number is zero or equal to the previous
   * one. An internal counter keeps track of missed frames.
   *
   * @return False if more than 2 frames were missed
   */
  bool checkMocapData();

  // getters

  virtual const cpp_msg::Mocap_msg &getPose() { return pose_; }

  virtual const std::string &getId() { return id_; }

  // console
  void consoleDebug(std::string &log, const std::string &message);
  void consoleInformation(std::string &log, const std::string &message);
  void consoleWarning(std::string &log, const std::string &message);
  void consoleError(std::string &log, const std::string &message);

protected:
  std::string id_ = "N/A";
  std::string *log_{nullptr};
  ConsoleState console_state_{console_debug};

  cpp_msg::Mocap_msg pose_{};
  int missed_frames_{0};
  int old_frame_number_{0};
};

} // namespace raptor