#include "logger.h"
#include "MocapPubSubTypes.h"
#include "csv_helper.h"
#include "domain_participant.h"
#include "sensor_msgs/msgs/Mocap.h"
#include "subscriber.h"
#include <chrono>
#include <thread>

void startLog(std::atomic<LogFlag> &log_flag,
              const std::string sub_topic_name) {
  std::unique_ptr<DefaultParticipant> dp =
      std::make_unique<DefaultParticipant>(0, "raptor");
  DDSSubscriber<idl_msg::MocapPubSubType, cpp_msg::Mocap> *mocap_sub;
  cpp_msg::Mocap pose{};
  mocap_sub = new DDSSubscriber(idl_msg::MocapPubSubType(), &pose,
                                sub_topic_name, dp->participant());

  // set delay for 100 Hz
  const int DELAY = 10;
  // declare chrono variables
  std::chrono::time_point<std::chrono::high_resolution_clock> time_0;
  std::chrono::time_point<std::chrono::high_resolution_clock> time_1;
  std::chrono::time_point<std::chrono::high_resolution_clock> loop_timer;
  std::chrono::duration<float, std::milli> duration;
  // declare container vectors
  std::vector<float> timestamp;
  std::vector<float> pos_x;
  std::vector<float> pos_y;
  std::vector<float> pos_z;

  // DEBUG
  std::cout << "Start log" << std::endl;

  // 'start' the clock
  time_0 = std::chrono::high_resolution_clock::now();
  while (true) {
    loop_timer = std::chrono::high_resolution_clock::now();
    switch (log_flag.load()) {
    case 0: // run
      break;

    case 1: { // stop
      // delete dynamic variables
      delete mocap_sub;
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
        year.push_back(date.at(i));
      }
      std::string month;
      for (int i = 4; i < 7; ++i) {
        month.push_back(date.at(i));
      }
      std::string day;
      for (int i = 8; i < 10; ++i) {
        day.push_back(date.at(i));
      }
      std::string time;
      for (int i = 11; i < 13; ++i) {
        time.push_back(date.at(i));
      }
      for (int i = 14; i < 16; ++i) {
        time.push_back(date.at(i));
      }
      // concatenate filname
      std::string filename = "log_" + month + day + '_' + year + '_' + time;
      // safe to file
      write_col_vec_to_csv(container, filename, ',', 1.f);
      // exit
      std::cout << "Successful log" << std::endl;
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
    pos_x.push_back(pose.pose.position.x);
    pos_y.push_back(pose.pose.position.y);
    pos_z.push_back(pose.pose.position.z);

    // wait for remaining time
    loop_timer += std::chrono::milliseconds(DELAY);
    std::this_thread::sleep_until(loop_timer);
  }
}