/*
  Provides funcionality to
  - get the current date and/or time,
    parsed from the system clock
  - save a log file with the proper naming
*/
#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>

/**
 * @brief Get the current time as a string, parsed from the system clock.
 *
 * @return std::string current_time
 */
std::string getTime() {
  std::time_t intermediate =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string raw_timestring = std::ctime(&intermediate);
  std::string result;
  // parse time
  for (int i = 11; i < 19; ++i) {
    result += raw_timestring.at(i);
  }

  return result;
}

/**
 * @brief Get the current date and time as a string formatted as
 * YYYYMMMDD_HH:MM:SS, parsed from the system clock.
 *
 * @return std::string current date and time
 */
std::string getDateTime() {
  std::time_t intermediate =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string raw_timestring = std::ctime(&intermediate);
  std::string result;
  // parse year
  for (int i = 20; i < 24; ++i) {
    result += raw_timestring.at(i);
  }
  // parse month
  for (int i = 4; i < 7; ++i) {
    result += raw_timestring.at(i);
  }
  // parse day
  for (int i = 8; i < 10; ++i) {
    if (i == 8 && raw_timestring.at(i) == ' ') {
      continue;
    }
    result += raw_timestring.at(i);
  }
  result += '_';
  // parse time
  for (int i = 11; i < 19; ++i) {
    result += raw_timestring.at(i);
  }

  return result;
}

/**
 * @brief Finds (or creates, if necessary) a requested path and a suitable
 * filename.
 *
 * @param[in] path Path to requested directory including a trailing slash (rel:
 * ./path)
 * @param[in] filename Requested name of the file
 * @param[in] type Ending type of the file (e.g. ".txt")
 * @return std::string path
 */
std::string getPath(const std::string &path, const std::string &filename,
                    const std::string &type) {
  // create directories if they don't exist already
  if (!std::filesystem::exists(path)) {
    std::filesystem::create_directories(path);
  }

  std::string file_path(path + filename);

  // find suitable filename
  std::filesystem::path fs_path{file_path + type};
  for (int i = 1; std::filesystem::exists(fs_path); ++i) {
    // don't pop-back number the first time
    if (i != 1) {
      file_path.pop_back();
      file_path.pop_back();
    }
    file_path += '_' + std::to_string(i);
    fs_path = file_path + type;
    if (i >= 100) {
      std::cerr << "Problem when creating file: Pathname exists over 100 times"
                << std::endl;
      exit(1); // TODO
    }
  }
  file_path += type;

  return file_path;
}

/**
 * @brief Saves the log file with the relative path
 * ./logs/console/consoleLog_YYYYMMMDD_HH:MM:SS.txt
 *
 * @param text
 * @return bool success
 */
bool saveLog(const std::string &text) {
  // create file
  std::ofstream ofs;
  ofs.open(getPath("./logs/console/", "consoleLog_" + getDateTime(), ".txt"));
  // check
  if (!ofs.is_open()) {
    std::cerr << "Problem when creating file: Couldn't open ofs." << std::endl;
    return false;
  }
  ofs << text;
  ofs.close();
  return true;
}