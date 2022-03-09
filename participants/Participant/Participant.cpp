#include "Participant.h"

bool raptor::Participant::checkForData()
{
  // check if subscriber matched something
  if (!mocap_sub_->listener->matched()) {
    std::cout << "[ERROR][Participant: " << id
              << "] Mocap subscriber is not matched." << std::endl;
    return false;
  }

  // read 10 data points
  float x;
  for (int i = 0; i < 10; ++i) {
    mocap_sub_->listener->wait_for_data();
    x = pose_.pose.position.x;
  }

  // check the last datapoint
  const float tolerance = 0.1;
  if (pose_.pose.position.x < tolerance && pose_.pose.position.y < tolerance &&
      pose_.pose.position.z < tolerance) {
    std::cout << "[ERROR][Participant: " << id << "] "
              << "Bad data after initializiation of mocap_subscriber."
              << std::endl;
    return false;
  }
  std::cout << "[INFO][Participant: " << id << "] "
            << "Good data after initializiation of mocap_subscriber."
            << std::endl;

  return true;
}

const cpp_msg::Mocap &raptor::Participant::getPose() { return pose_; }

const std::string &raptor::Participant::getId() { return id; }