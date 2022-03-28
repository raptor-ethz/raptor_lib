#include "Participant.h"

bool raptor::Participant::checkMocapData() {
  // read frame number
  long frame_number = pose_.header.timestamp;
  // run checks
  if (frame_number == 0 || frame_number == old_frame_number_) {
    ++missed_frames_; // increment if bad data
  } else {
    missed_frames_ = 0; // reset if good data
  }
  // update old frame number
  old_frame_number_ = frame_number;
  // check for 3 consecutive missed frames
  if (missed_frames_ > 2) {
    // error
    std::cout << "[ERROR][Participant: " << id_
              << "] Bad motion capture data detected." << std::endl;
    return false;
  }

  return true;
}

bool raptor::Participant::initializeMocapSub() {
  // info
  std::cout << "[INFO][Participant: " << id_ << "] "
            << "Initializing mocap subscriber." << std::endl;

  // check if subscriber matched for max 10 times
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

  // attempt to receive 10 data points (for max 500ms each)
  for (int i = 0; i < 10; ++i) {
    if (!mocap_sub_->listener->matched()) {
      // error if subscriber unmatched
      std::cout << "[ERROR][Participant: " << id_
                << "] Mocap subscriber unmatched." << std::endl;
      return false;
    }
    mocap_sub_->listener->wait_for_data_for_ms(500);
  }

  for (int i = 0;; ++i) {
    // check data quality
    for (int j = 0; j < 2; ++j) {
      checkMocapData();
      mocap_sub_->listener->wait_for_data_for_ms(100);
    }
    if (!checkMocapData()) {
      // error
      std::cout << "[ERROR][Participant: " << id_
                << "] Bad motion capture data." << std::endl;
      // return if it is the last try
      if (i == 4) {
        // error
        std::cout << "[ERROR][Participant: " << id_
                  << "] Initialization failed: Bad motion capture data."
                  << std::endl;
        return false;
      }
      // rerun checks
      std::cout
          << "Rerunning Mocap Initialization in 3 seconds (remaining tries: "
          << 4 - i << ")." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      continue;
    }
  }

  // info for good data
  std::cout << "[INFO][Participant: " << id_ << "] "
            << "Good data after initializiation of mocap subscriber."
            << std::endl;
  return true;
}