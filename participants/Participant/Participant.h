#pragma once

#include "Mocap_msg.h"
#include "Mocap_msgPubSubTypes.h"

#include "domain_participant.h"
#include "publisher.h"
#include "subscriber.h"

#include <atomic>

namespace raptor {

class Participant {
public:
  DDSSubscriber<idl_msg::Mocap_msgPubSubType, cpp_msg::Mocap_msg> *mocap_sub_;

  /**
   * Checks if the subscriber has matched anything and
   * if the received data is non-zero.
   *
   * @returns If subscriber is matched and received non-zero data.
   **/
  bool checkForData();

  virtual const cpp_msg::Mocap_msg &getPose() { return pose_; }

  virtual const std::string &getId() { return id_; }

protected:
  std::string id_ = "N/A";
  cpp_msg::Mocap_msg pose_{};
};

} // namespace raptor