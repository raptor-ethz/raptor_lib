#pragma once

#include "MocapPubSubTypes.h"
#include "QuadPositionCmdPubSubTypes.h"
#include "domain_participant.h"
#include "geometry_msgs/msgs/Position.h"
#include "publisher.h"
#include "quadcopter_msgs/msgs/QuadPositionCmd.h"
#include "sensor_msgs/msgs/Mocap.h"
#include "std_msgs/msgs/Header.h"
#include "subscriber.h"
#include <atomic>

namespace raptor
{

class Participant
{
public:
  DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> *mocap_sub_;

  /**
   * Checks if the subscriber has matched anything and
   * if the received data is non-zero.
   *
   * @returns true : if subscriber is matched and received non-zero data.
   **/
  bool checkForData();

  virtual const cpp_msg::Mocap &getPose() { return pose_; }

  virtual const std::string &getId() { return id_; }

protected:
  std::string id_ = "N/A";
  cpp_msg::Mocap pose_{};
};

} // namespace raptor