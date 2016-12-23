#ifndef LOGGER_H
#define LOGGER_H

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <ros2log/sink.hpp>

#include <chrono>

enum class Log_Levels { DEBUG, INFO, WARN, ERROR, FATAL, NONE };

class Logger {
 public:
  Logger(std::shared_ptr<rclcpp::node::Node> n) : node(n) {
    std::vector<rclcpp::parameter::ParameterVariant> params;
    params.emplace_back("log_level", "INFO");
    node->set_parameters(params);

    node->register_param_change_callback([=](
        const std::vector<rclcpp::parameter::ParameterVariant>& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (auto param : parameters) {
        for (auto& sink : sinks) {
          if (sink.name + "_level" == param.get_name()) {
            if (param.get_type() ==
                rclcpp::parameter::ParameterType::PARAMETER_STRING) {
              std::string level_str = param.get_value<std::string>();

              if (level_str == "DEBUG")
                sink.output_level = Log_Levels::DEBUG;

              else if (level_str == "INFO")
                sink.output_level = Log_Levels::INFO;

              else if (level_str == "WARN")
                sink.output_level = Log_Levels::WARN;

              else if (level_str == "ERROR")
                sink.output_level = Log_Levels::ERROR;

              else if (level_str == "FATAL")
                sink.output_level = Log_Levels::FATAL;

              else if (level_str == "NONE")
                sink.output_level = Log_Levels::NONE;

              else {
                result.successful = false;
                result.reason =
                    "log_level must be one of: DEBUG, INFO, WARN, ERROR, "
                    "FATAL, "
                    "NONE";
              }
            } else {
              result.successful = false;
              result.reason = "log_level must be a string";
            }
          }
        }
      }

      return result;
    });
  };

  Logger() = default;
  ~Logger() = default;

  virtual void register_sink(const Sink& sink) { sinks.push_back(sink); }

 protected:
  std::shared_ptr<rclcpp::node::Node> node;

  std::vector<Sink> sinks;

  virtual void output(Log_Levels level, const char* log_string) const {
    for (auto& sink : sinks) {
      if (level >= sink.output_level) {
        sink.output_function(level, 
          add_metadata(level_to_string(level), log_string).c_str());
      }
    }
  }

  static const char * level_to_string(Log_Levels level) {
    switch(level) {
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
  std::string add_metadata(const char * level, 
                           const char * data) const {

    auto now = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    now -= seconds;
    auto nano_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now);

    auto secs = seconds.count();
    auto nsecs = nano_seconds.count();

    return fmt::format("{level:<5}: [{secs}.{nsecs:0<9}] {data}",
                       FMT_CAPTURE(level, secs, nsecs, data));
  }
};

#endif