#ifndef FMT_LOGGER_H
#define FMT_LOGGER_H

#include <fmt/format.h>
#include <ros2log/logger.hpp>

class FmtLogger : public Logger {
 public:
  using Logger::Logger;

  template <typename... Args>
  void debug(const char *file, const char *function, int line, const char *fmt,
             Args &&... args) const {
    auto md = MetaData{file, function, line};
    auto data = fmt::format(fmt, std::forward<Args>(args)...);
    output(Log_Levels::DEBUG, md, data.c_str());
  }

  template <typename... Args>
  void info(const char *file, const char *function, int line, const char *fmt,
            Args &&... args) const {
    auto md = MetaData{file, function, line};
    auto data = fmt::format(fmt, std::forward<Args>(args)...);
    output(Log_Levels::INFO, md, data.c_str());
  }

  template <typename... Args>
  void warn(const char *file, const char *function, int line, const char *fmt,
            Args &&... args) const {
    auto md = MetaData{file, function, line};
    auto data = fmt::format(fmt, std::forward<Args>(args)...);
    output(Log_Levels::WARN, md, data.c_str());
  }

  template <typename... Args>
  void error(const char *file, const char *function, int line, const char *fmt,
             Args &&... args) const {
    auto md = MetaData{file, function, line};
    auto data = fmt::format(fmt, std::forward<Args>(args)...);
    output(Log_Levels::ERROR, md, data.c_str());
  }

  template <typename... Args>
  void fatal(const char *file, const char *function, int line, const char *fmt,
             Args &&... args) const {
    auto md = MetaData{file, function, line};
    auto data = fmt::format(fmt, std::forward<Args>(args)...);
    output(Log_Levels::FATAL, md, data.c_str());
  }
};

#endif