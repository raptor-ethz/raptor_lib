#include "fshelper.h"

#include <filesystem>
#include <fstream>
#include <iostream>

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