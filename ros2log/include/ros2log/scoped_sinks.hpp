#ifndef SCOPED_SINKS_H
#define SCOPED_SINKS_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2log/logger.hpp>
#include <rosgraph_msgs/msg/log.hpp>

#include <fmt/format.h>

// Sink that publishes to rosout, and is attached as long as it is in scope
class ScopedRosoutSink {
 public:
  ScopedRosoutSink(std::shared_ptr<Logger> logger,
                   std::shared_ptr<rclcpp::node::Node> n)
      : logger_(logger), node(n) {
    rosout_pub = node->create_publisher<rosgraph_msgs::msg::Log>("rosout");
    logger_->register_sink(
        Sink("rosout", Log_Levels::INFO, [this](LogMessage message) {
          auto now = message.timestamp.time_since_epoch();
          auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
          now -= seconds;
          auto nano_seconds =
              std::chrono::duration_cast<std::chrono::nanoseconds>(now);

          auto secs = seconds.count();
          auto nsecs = nano_seconds.count();

          auto msg = rosgraph_msgs::msg::Log();
          msg.header.stamp.sec = secs;
          msg.header.stamp.nanosec = nsecs;
          msg.level = static_cast<uint8_t>(message.level);
          msg.name = message.logger_name;  // TODO(rohbotics) use the path here
          msg.file = message.file;
          msg.function = message.function;
          msg.line = message.line;
          msg.msg = message.log_string;
          rosout_pub->publish(msg);
        }));
  };

  ~ScopedRosoutSink() { logger_->deregister_sink("rosout"); };

 private:
  std::shared_ptr<Logger> logger_;

  std::shared_ptr<rclcpp::node::Node> node;
  rclcpp::Publisher<rosgraph_msgs::msg::Log>::SharedPtr rosout_pub;
};

// Sink that prints in color, and is attached as long as it is in scope
class ScopedPrintSink {
 public:
  ScopedPrintSink(std::shared_ptr<Logger> logger) : logger_(logger) {
    logger_->register_sink(
        Sink("print", Log_Levels::INFO, [](LogMessage message) {
          auto now = message.timestamp.time_since_epoch();
          auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
          now -= seconds;
          auto nano_seconds =
              std::chrono::duration_cast<std::chrono::nanoseconds>(now);

          auto secs = seconds.count();
          auto nsecs = nano_seconds.count();

          auto logger_name = message.logger_name;
          auto level = level_to_string(message.level);
          auto file = message.file;
          auto function = message.function;
          auto line = message.line;
          auto data = message.log_string;

          auto log_string =
              fmt::format("{level:<5}: [{secs}.{nsecs:0<9}] {data}",
                          FMT_CAPTURE(level, secs, nsecs, data, file, function,
                                      line, logger_name));

          switch (message.level) {
            case Log_Levels::FATAL:
            case Log_Levels::ERROR:
              fprintf(stderr, "\x1b[31m%s\x1b[0m\n", log_string.c_str());
              break;
            case Log_Levels::WARN:
              fprintf(stderr, "\x1b[33m%s\x1b[0m\n", log_string.c_str());
              break;
            case Log_Levels::DEBUG:
              printf("\x1b[32m%s\x1b[0m\n", log_string.c_str());
              break;
            case Log_Levels::INFO:
              printf("%s\n", log_string.c_str());
              break;
          }
        }));
  };

  ~ScopedPrintSink() { logger_->deregister_sink("print"); };

 private:
  std::shared_ptr<Logger> logger_;

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
};

#endif