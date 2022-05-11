#include "Item.h"

Item::Item(const std::string &raptor_participant_id,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name)
{
  id_ = raptor_participant_id;

  mocap_sub_ = new DDSSubscriber(idl_msg::Mocap_msgPubSubType(), &pose_,
                                 sub_topic_name, dp->participant());
};

Item::Item(const std::string &raptor_participant_id, std::string *const log,
           std::unique_ptr<DefaultParticipant> &dp,
           const std::string &sub_topic_name)
{
  id_ = raptor_participant_id;
  log_ = log;

  mocap_sub_ = new DDSSubscriber(idl_msg::Mocap_msgPubSubType(), &pose_,
                                 sub_topic_name, dp->participant());
};

Item::~Item() { delete mocap_sub_; }

bool Item::setInitialPosition()
{
  float x, y, z;

  // try 5 times to receive good data
  for (int i = 0; i < 5; ++i)
  {
    // read position data
    x = pose_.position.x;
    y = pose_.position.y;
    z = pose_.position.z;
    // check position data
    if (!(x < 0.001 && y < 0.001 && z < 0.001))
    {
      initial_position_x_ = x;
      initial_position_y_ = y;
      initial_position_y_ = z;
      consoleInformation("Good Initial Position saved.");
      return true;
    }
    // wait a bit before the next attempt
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  consoleError("Bad Initial Position.");
  return false;
}

float Item::getInitialPositionX() { return initial_position_x_; }

float Item::getInitialPositionY() { return initial_position_y_; }

float Item::getInitialPositionZ() { return initial_position_z_; }

std::vector<float> Item::getPoseAsVector()
{
  std::vector<float> return_value;
  return_value.push_back(pose_.position.x);
  return_value.push_back(pose_.position.y);
  return_value.push_back(pose_.position.z);
  return return_value;
}