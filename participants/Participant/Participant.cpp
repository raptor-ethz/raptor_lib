#include "Participant.h"

bool raptor::Participant::checkMocapData() {
  // read frame number
  int frame_number = pose_.header.timestamp;
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
    // warning
    consoleWarning("Bad motion capture data detected.");
    return false;
  }

  return true;
}

bool raptor::Participant::initializeMocapSub() {
  consoleInformation("Initializing motion capture subscriber");
  // define max tries
  const int n = 5;

  // check if subscriber matched for max 5 times (3 seconds each)
  for (int i = 1; !mocap_sub_->listener->matched(); ++i) {
    if (i == n) {
      // error if subscriber didn't match after 10 tries
      consoleError("Failed to match motion capture subscriber.");
      return false;
    }
    consoleWarning("Motion capture subscriber did not match. Check that "
                   "mocap_pub is running.");
    consoleWarning("Rerunning initialization in 3 seconds (remaining tries: " +
                   std::to_string(n - i) + ").");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
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
    // mocap_sub_->listener->wait_for_data(); TODO
  }

  for (int j = 0; j < 2; ++j) {
    mocap_sub_->listener->wait_for_data_for_ms(100);
    checkMocapData();
  }
  for (int i = 0;; ++i) {
    mocap_sub_->listener->wait_for_data_for_ms(100);
    // check data quality
    if (checkMocapData()) {
      break;
    }
    // return if it is the last try
    if (i == 4) {
      // error
      std::cout << "[ERROR][Participant: " << id_
                << "] Initialization failed: Bad motion capture data (check "
                   "that vicon stream is live)."
                << std::endl;
      return false;
    }
    // warning -> rerun checks
    std::cout
        << "[WARNING][Participant: " << id_
        << "] Bad motion capture data (check that vicon stream is live). "
           "Rerunning mocap initialization in 3 seconds (remaining tries: "
        << 4 - i << ")." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  // info for good data
  std::cout << "[INFO][Participant: " << id_ << "] "
            << "Good data after initializiation of mocap subscriber."
            << std::endl;
  return true;
}