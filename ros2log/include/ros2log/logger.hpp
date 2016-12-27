#ifndef LOGGER_H
#define LOGGER_H

#include <fmt/format.h>
#include <ros2log/sink.hpp>

#include <algorithm>
#include <chrono>

#ifndef NDEBUG
#ifndef LOG_DEBUG
#define LOG_DEBUG(_logger, ...) \
  _logger->debug(__FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif
#else
#ifndef LOG_DEBUG
#define LOG_DEBUG(_logger, ...)
#endif
#endif

#ifndef LOG_INFO
#define LOG_INFO(_logger, ...) \
  _logger->info(__FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_WARN
#define LOG_WARN(_logger, ...) \
  _logger->warn(__FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_ERROR
#define LOG_ERROR(_logger, ...) \
  _logger->error(__FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

#ifndef LOG_FATAL
#define LOG_FATAL(_logger, ...) \
  _logger->fatal(__FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
#endif

enum class Log_Levels { DEBUG = 1, INFO = 2, WARN = 4, ERROR = 8, FATAL = 16 };

struct MetaData {
  MetaData(std::chrono::time_point<std::chrono::system_clock> _timestamp,
           const char* _file, const char* _function, int _line)
      : timestamp(_timestamp), file(_file), function(_function), line(_line){};

  MetaData() = default;

  std::chrono::time_point<std::chrono::system_clock> timestamp;
  const char* file = nullptr;
  const char* function = nullptr;
  int line = 0;
};

class Logger {
 public:
  Logger() = default;
  ~Logger() = default;

  virtual void register_sink(const Sink& sink) { sinks.push_back(sink); }

  virtual void deregister_sink(const std::string& name) {
    sinks.erase(std::remove_if(sinks.begin(), sinks.end(), [name](Sink sink) {
      return sink.name == name;
    }));
  }

  virtual std::vector<Sink>& get_sinks() { return sinks; }

 protected:
  std::vector<Sink> sinks;

  virtual void output(Log_Levels level, MetaData md,
                      const char* log_string) const {
    for (auto& sink : sinks) {
      if (sink.enabled && level >= sink.output_level) {
        sink.output_function(
            level, md,
            add_metadata(level_to_string(level), md, log_string).c_str());
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
  std::string add_metadata(const char* level, MetaData md,
                           const char* data) const {
    auto now = md.timestamp.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto secs = seconds.count();
    auto nsecs = nano_seconds.count();

    auto file = md.file;
    auto function = md.function;
    auto line = md.line;

    return fmt::format(
        "{level:<5}: [{secs}.{nsecs:0<9}] {data}",
        FMT_CAPTURE(level, secs, nsecs, data, file, function, line));
  }
};

#endif