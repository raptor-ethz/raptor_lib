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
    consoleWarning("Bad motion capture data detected.");
    return false;
  }

  return true;
}

bool raptor::Participant::initializeMocapSub() {
  consoleInformation("Initializing motion capture subscriber");

  // check if subscriber matched for max n times (3 seconds each)
  // define max tries
  const int n = 5;
  for (int i = 1; !mocap_sub_->listener->matched(); ++i) {
    // return error after the n-th try
    if (i == n) {
      consoleError("Initialization failed: Failed to match motion capture subscriber.");
      return false;
    }
    // rerun checks
    consoleWarning("Motion capture subscriber did not match (check that "
                   "mocap_publisher is running).");
    consoleWarning("Rerunning initialization in 3 seconds (remaining tries: " +
                   std::to_string(n - i) + ").");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  // attempt to receive 10 data points (for max 500ms each)
  for (int i = 0; i < 10; ++i) {
    // check that subscriber is matched, return error otherwise
    if (!mocap_sub_->listener->matched()) {
      consoleError("Motion capture subscriber unmatched.");
      return false;
    }
    mocap_sub_->listener->wait_for_data_for_ms(500);
  }

  for (int j = 0; j < 2; ++j) {
    mocap_sub_->listener->wait_for_data_for_ms(100);
    checkMocapData();
  }

  // check data quality for max n times (3 seconds each)
  // use max tries from above
  for (int i = 1;; ++i) {
    mocap_sub_->listener->wait_for_data_for_ms(100);
    // break if data is good
    if (checkMocapData()) {
      break;
    }
    // return error after the n-th try
    if (i == n) {
      consoleError("Initialization failed: Bad motion capture data.");
      return false;
    }
    // rerun checks
    consoleWarning("Received bad motion capture data (check that vicon stream is live).");
    consoleWarning("Rerunning data quality checks in 3 seconds (remaining tries: " +
                   std::to_string(n - i) + ").");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }

  // good data
  consoleInformation("Good motion capture data received after initialization.");
  return true;
}