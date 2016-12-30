#ifndef LOGGER_H
#define LOGGER_H

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
  LogMessage(const char* _logger_name, Log_Levels _level,
             std::chrono::time_point<std::chrono::system_clock> _timestamp,
             const char* _file, const char* _function, int _line,
             const char* _log_string)
      : logger_name(_logger_name),
        level(_level),
        timestamp(_timestamp),
        file(_file),
        function(_function),
        line(_line),
        log_string(_log_string){};

  LogMessage() = default;

  // Metadata about the log message
  const char* logger_name = nullptr;  
  Log_Levels level;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  const char* file = nullptr;
  const char* function = nullptr;
  int line = 0;

  // The actual string that we want to log
  const char* log_string = nullptr;
};

class Logger {
 public:
  Logger(const std::string& logger_name = "",
         std::shared_ptr<Logger> logger_parent = nullptr)
      : name(logger_name), parent(logger_parent) {
        if (parent != nullptr) {
          parent->register_child(this);
        }
      };

  ~Logger() {
    if (parent != nullptr) {
      parent->deregister_child(logger_id);
    }
  };

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

  virtual std::vector<Logger*>& get_children() {
    return children;
  }


  virtual void set_name(const std::string& _name) {
    name = _name;
  }

  virtual std::string get_name() {
    return name;
  }

 protected:
  std::vector<Sink> sinks;
  std::string name;

  std::shared_ptr<Logger> parent;
  int logger_id = -1;
  std::vector<Logger*> children;

  virtual void output(LogMessage& message) const {
    if (parent) {
      parent->output(message);
      return;
    }
    for (auto& sink : sinks) {
      if (sink.enabled && message.level >= sink.output_level) {
        sink.output_function(message);
      }
    }
  }

  virtual void register_child(Logger* child) {
    child->logger_id = children.size();
    children.push_back(child);
  }

  virtual void deregister_child(int child_id) {
    children.erase(std::remove_if(children.begin(), children.end(), [child_id](Logger* child) {
      return child->logger_id == child_id;
    }));
  }
};

#endif