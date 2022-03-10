#include "Participant.h"
#include "csv_helper.h"
#include <chrono>

void log(raptor::Participant participant)
{
  // set delay for 10 Hz
  const int DELAY = 100;
  // declare chrono variables
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
  time_0 = std::chrono::high_resolution_clock::now();
  while (true) {
    switch (participant.log_flag_.load()) {
    case 0: // run
      break;

    case 1: { // stop
      // unite column vectors into container vector
      std::vector<std::vector<float>> container;
      container.push_back(timestamp);
      container.push_back(pos_x);
      container.push_back(pos_y);
      container.push_back(pos_z);
      // create filename
      std::time_t timestamp = std::chrono::system_clock::to_time_t(
          std::chrono::system_clock::now());
      std::string date = std::ctime(&timestamp);
      // parse date string
      std::string year;
      for (int i = 20; i < 24; ++i) {
        std::cout << date.at(i) << std::endl;
        year.push_back(date.at(i));
      }
      std::string month;
      for (int i = 4; i < 7; ++i) {
        std::cout << date.at(i) << std::endl;
        month.push_back(date.at(i));
      }
      std::string day;
      for (int i = 8; i < 10; ++i) {
        std::cout << date.at(i) << std::endl;
        day.push_back(date.at(i));
      }
      std::string time;
      for (int i = 11; i < 13; ++i) {
        std::cout << date.at(i) << std::endl;
        time.push_back(date.at(i));
      }
      for (int i = 14; i < 16; ++i) {
        std::cout << date.at(i) << std::endl;
        time.push_back(date.at(i));
      }
      // concatenate filname
      std::string filename = "log_" + month + day + '_' + year + '_' + time;
      // safe to file
      write_col_vec_to_csv(container, filename, ',', 1.f);
      // exit
      return;
    }

      // case 2 to be handled (bookmark)

    default:
      break;
    }

    // store timestamp
    time_1 = std::chrono::high_resolution_clock::now();
    duration = time_1 - time_0;
    timestamp.push_back(duration.count());
    // store position data
    pos_x.push_back(participant.getPose().pose.position.x);
    pos_y.push_back(participant.getPose().pose.position.y);
    pos_z.push_back(participant.getPose().pose.position.z);

    // wait for remaining time
    time_2 = std::chrono::high_resolution_clock::now();
    duration = time_2 - time_1;
    int wait = DELAY - duration.count();
    if (wait > 5) {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait));
    }
  }
}