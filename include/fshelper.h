#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>

std::string parseDateAndTime() {
  std::string result;
  std::string raw = __DATE__;
  // parse year
  for (int i = 7; i < 11; ++i) {
    result += raw.at(i);
  }
  // parse month and day
  for (int i = 0; i < 6; ++i) {
    if (i == 3) {
      continue;
    }
    if (i == 4 && raw.at(4) == ' ') {
      result += '0';
      continue;
    }
    result += raw.at(i);
  }
  // add time
  result += '_' + std::string(__TIME__);

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
                << std::endl; //TODO
      exit(1);
    }
  }
  file_path += type;

  return file_path;
}