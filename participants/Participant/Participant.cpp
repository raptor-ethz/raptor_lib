#include "Participant.h"

bool raptor::Participant::checkForData() {
  // check if subscriber matched for 10 times
  for (int i = 0;; ++i) {
    // exit loop if matched
    if (mocap_sub_->listener->matched()) {
      break;
    }
    if (i == 9) {
      // error if subscriber didn't match after 10 tries
      std::cout << "[ERROR][Participant: " << id_
                << "] Mocap subscriber did not match." << std::endl;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // read 10 data points
  for (int i = 0; i < 10; ++i) {
    if (!mocap_sub_->listener->matched()) {
      // error if subscriber unmatched
      std::cout << "[ERROR][Participant: " << id_
                << "] Mocap subscriber unmatched." << std::endl;
      return false;
    }
    mocap_sub_->listener->wait_for_data_for_ms(2000);
  }
  
  // check the last datapoint
  if (pose_.header.timestamp == 0 ) {
    std::cout << "[ERROR][Participant: " << id_ << "] "
              << "Bad data after initializiation of mocap subscriber."
              << std::endl;
    return false;
  }
  std::cout << "[INFO][Participant: " << id_ << "] "
            << "Good data after initializiation of mocap subscriber."
            << std::endl;
  return true;
}