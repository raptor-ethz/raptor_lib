#include "Participant.h"
#include <chrono>

int log(raptor::Participant participant)
{
  // set delay for 10 Hz
  const int DELAY = 100;
  // declare chrono variables
  std::chrono::high_resolution_clock clock;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_0;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_1;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_2;
  std::chrono::duration<float, std::milli> duration;
  // declare container vectors
  std::vector<float> timestamp;
  std::vector<float> pos_x;
  std::vector<float> pos_y;
  std::vector<float> pos_z;

  // 'start' the clock
  time_0 = clock.now();
  while (true) {
    switch (participant.log_flag_) {
    case 0: // run
      break;

    case 1: // stop
      // safe to file
      // exit
      break;

      // case 2 to be handled (bookmark)

    default:
      break;
    }

    // store timestamp
    time_1 = clock.now();
    duration = time_1 - time_0;
    timestamp.push_back(duration.count());
    // store position data
    pos_x.push_back(participant.getPose().pose.position.x);
    pos_y.push_back(participant.getPose().pose.position.y);
    pos_z.push_back(participant.getPose().pose.position.z);

    // wait for remaining time
    time_2 = clock.now();
    duration = time_2 - time_1;
    int wait = DELAY - duration.count();
    if (wait > 5) {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait));
    }
  }

  return 0;
}