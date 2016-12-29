#ifndef LOGGER_H
#define LOGGER_H

#include <fmt/format.h>
#include <ros2log/sink.hpp>

#include <algorithm>
#include <chrono>
#include <memory>

#ifndef NDEBUG
#ifndef LOG_DEBUG
#define LOG_DEBUG(_logger, ...) \
  _logger->log(Log_Levels::DEBUG, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif
#else
#ifndef LOG_DEBUG
#define LOG_DEBUG(_logger, ...)
#endif
#endif

#ifndef LOG_INFO
#define LOG_INFO(_logger, ...) \
  _logger->log(Log_Levels::INFO, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_WARN
#define LOG_WARN(_logger, ...) \
  _logger->log(Log_Levels::WARN, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_ERROR
#define LOG_ERROR(_logger, ...) \
  _logger->log(Log_Levels::ERROR, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_FATAL
#define LOG_FATAL(_logger, ...) \
  _logger->log(Log_Levels::FATAL, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

enum class Log_Levels { DEBUG = 1, INFO = 2, WARN = 4, ERROR = 8, FATAL = 16 };

struct LogMessage {
  LogMessage(Log_Levels _level,
             std::chrono::time_point<std::chrono::system_clock> _timestamp,
             const char* _file, const char* _function, int _line,
             std::string _log_string)
      : level(_level),
        timestamp(_timestamp),
        file(_file),
        function(_function),
        line(_line),
        log_string(_log_string){};

  LogMessage() = default;

  // Metadata about the log message
  Log_Levels level;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  const char* file = nullptr;
  const char* function = nullptr;
  int line = 0;

  // The actual string that we want to log
  std::string log_string;
};

class Logger {
 public:
  Logger(const std::string& logger_name = "",
         std::shared_ptr<Logger> logger_parent = nullptr)
      : name(logger_name), parent(parent){};
  ~Logger() = default;

  virtual void register_sink(const Sink& sink) {
    if (parent != nullptr) {
      parent->register_sink(sink);
      return;
    }
    sinks.push_back(sink);
  }

  virtual void deregister_sink(const std::string& name) {
    if (parent != nullptr) {
      parent->deregister_sink(name);
      return;
    }
    sinks.erase(std::remove_if(sinks.begin(), sinks.end(), [name](Sink sink) {
      return sink.name == name;
    }));
  }

  virtual std::vector<Sink>& get_sinks() {
    if (parent != nullptr) {
      return parent->get_sinks();
    }
    return sinks;
  }

  virtual std::string get_name() {
    if (parent != nullptr) {
      return parent->get_name();
    }
    return name;
  }

 protected:
  std::vector<Sink> sinks;
  std::string name;

  std::shared_ptr<Logger> parent;

  virtual void output(LogMessage message) const {
    if (parent != nullptr) {
      parent->output(message);
      return;
    }

    for (auto& sink : sinks) {
      if (sink.enabled && message.level >= sink.output_level) {
        sink.output_function(message);
      }
    }
  }

  static const char* level_to_string(Log_Levels level) {
    switch (level) {
      case Log_Levels::FATAL:
        return "FATAL";
        break;
      case Log_Levels::ERROR:
        return "ERROR";
        break;
      case Log_Levels::WARN:
        return "WARN";
        break;
      case Log_Levels::DEBUG:
        return "DEBUG";
        break;
      case Log_Levels::INFO:
        return "INFO";
        break;
    }
  }

  // TODO Do this function without fmt
  // std::string add_metadata(const char* level, MetaData md,
  //                          const char* data) const {
  //   auto now = md.timestamp.time_since_epoch();
  //   auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
  //   now -= seconds;
  //   auto nano_seconds =
  //       std::chrono::duration_cast<std::chrono::nanoseconds>(now);

  //   auto secs = seconds.count();
  //   auto nsecs = nano_seconds.count();

  //   auto file = md.file;
  //   auto function = md.function;
  //   auto line = md.line;

  //   return fmt::format(
  //       "{level:<5}: [{secs}.{nsecs:0<9}] {data}",
  //       FMT_CAPTURE(level, secs, nsecs, data, file, function, line, name));
  // }
};

#endif