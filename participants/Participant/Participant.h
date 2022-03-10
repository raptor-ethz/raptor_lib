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

namespace raptor
{

enum LogFlag { run, stop, bookmark };

class Participant
{
public:
  DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> *mocap_sub_;

  /**
   * Reads 30 datapoints from the subscriber and
   * checks for a non-zero last datapoint.
   *
   * @returns if (position.x != 0.0)
   **/
  virtual bool checkForData();

  virtual const cpp_msg::Mocap &getPose();

  virtual const std::string &getId();

  LogFlag log_flag_ = run;

protected:
  std::string id = "N/A";
  cpp_msg::Mocap pose_{};
};

} // namespace raptor