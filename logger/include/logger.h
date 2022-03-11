#pragma once
#include <atomic>

enum LogFlag { run, stop, bookmark };

void startLog(std::atomic<LogFlag> &log_flag);