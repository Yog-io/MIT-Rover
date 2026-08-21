#pragma once

#include <iostream>
#include <mutex>
#include <string>
#include <chrono>
#include <iomanip>

class Logger {
public:
    static Logger& get_instance() {
        static Logger instance;
        return instance;
    }

    void log(const std::string& level, const std::string& tag, const std::string& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::system_clock::now();
        std::time_t time = std::chrono::system_clock::to_time_t(now);
        
        std::cout << "[" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << "] "
                  << "[" << level << "] [" << tag << "] " << msg << std::endl;
    }

private:
    Logger() = default;
    std::mutex mutex_;
};

#define LOG_INFO(tag, msg) Logger::get_instance().log("INFO", tag, msg)
#define LOG_WARN(tag, msg) Logger::get_instance().log("WARN", tag, msg)
#define LOG_ERROR(tag, msg) Logger::get_instance().log("ERROR", tag, msg)
