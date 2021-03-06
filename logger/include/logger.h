#pragma once
#include <atomic>
#include <string>

enum LogFlag { run, stop, bookmark };

void startLog(std::atomic<LogFlag> &log_flag, const std::string sub_topic_name);