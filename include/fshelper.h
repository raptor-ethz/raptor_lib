#pragma once
/*
  Provides funcionality to
  - get the current date and/or time,
    parsed from the system clock
  - save a log file with the proper naming
*/
#include <string>


/**
 * @brief Get the current time as a string, parsed from the system clock.
 *
 * @return std::string current_time
 */
extern std::string getTime();

/**
 * @brief Get the current date and time as a string formatted as
 * YYYYMMMDD_HH:MM:SS, parsed from the system clock.
 *
 * @return std::string current date and time
 */
extern std::string getDateTime();

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
extern std::string getPath(const std::string &path, const std::string &filename,
                    const std::string &type);

/**
 * @brief Saves the log file with the relative path
 * ./logs/console/consoleLog_YYYYMMMDD_HH:MM:SS.txt
 *
 * @param text
 * @return bool success
 */
extern bool saveLog(const std::string &text);