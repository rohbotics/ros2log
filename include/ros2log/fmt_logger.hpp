#ifndef FMT_LOGGER_H
#define FMT_LOGGER_H

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2log/logger.hpp>

class FmtLogger : public Logger {
 public:
  using Logger::Logger;

  template <typename... Args>
  void debug(const char *f, Args &&... args) const {
    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::DEBUG, data.c_str());
  }

  template <typename... Args>
  void info(const char *f, Args &&... args) const {
    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::INFO, data.c_str());
  }

  template <typename... Args>
  void warn(const char *f, Args &&... args) const {
    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::WARN, data.c_str());
  }

  template <typename... Args>
  void error(const char *f, Args &&... args) const {
    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::ERROR, data.c_str());
  }

  template <typename... Args>
  void fatal(const char *f, Args &&... args) const {
    auto data = fmt::format(f, std::forward<Args>(args)...);
    output(Log_Levels::FATAL, data.c_str());
  }

};

#endif