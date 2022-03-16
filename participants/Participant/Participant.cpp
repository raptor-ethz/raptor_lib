#include "Participant.h"

bool raptor::Participant::checkForData()
{
  // run subscriber function 10 times
  for (int i = 0; i < 10; ++i) {
    mocap_sub_->listener->matched();
  }

  // check if subscriber matched something
  if (!mocap_sub_->listener->matched()) {
    std::cout << "[ERROR][Participant: " << id_
              << "] Mocap subscriber did not match." << std::endl;
    return false;
  }

  // read 10 data points
  float x;
  for (int i = 0; i < 10; ++i) {
    mocap_sub_->listener->wait_for_data();
    x = pose_.pose.position.x; // TODO: necessary?
  }

  // check the last datapoint
  const float tolerance = 0.1;
  if (pose_.header.timestamp == 0) {
    std::cout << "[ERROR][Participant: " << id_ << "] "
              << "Bad data after initializiation of mocap_subscriber."
              << std::endl;
    return false;
  }
  std::cout << "[INFO][Participant: " << id_ << "] "
            << "Good data after initializiation of mocap_subscriber."
            << std::endl;

  return true;
}