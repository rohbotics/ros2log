#ifndef SCOPED_SINKS_H
#define SCOPED_SINKS_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2log/logger.hpp>

class ScopedRosoutSink {
 public:
  ScopedRosoutSink(std::shared_ptr<Logger> logger,
                   std::shared_ptr<rclcpp::node::Node> n)
      : logger_(logger), node(n) {
    rosout_pub = node->create_publisher<std_msgs::msg::String>("rosout");
    logger_->register_sink(
        Sink("rosout", Log_Levels::INFO,
             [this](Log_Levels level, MetaData md, const char* log_string) {
               auto message = std_msgs::msg::String();
               message.data = log_string;
               rosout_pub->publish(message);
             }));
  };

  ~ScopedRosoutSink() { logger_->deregister_sink("rosout"); };

 private:
  std::shared_ptr<Logger> logger_;

  std::shared_ptr<rclcpp::node::Node> node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rosout_pub;
};

class ScopedPrintSink {
 public:
  ScopedPrintSink(std::shared_ptr<Logger> logger) : logger_(logger) {
    logger_->register_sink(
        Sink("print", Log_Levels::INFO,
             [](Log_Levels level, MetaData md, const char* log_string) {
               switch (level) {
                 case Log_Levels::FATAL:
                 case Log_Levels::ERROR:
                   printf("\x1b[31m%s\x1b[0m\n", log_string);
                   break;
                 case Log_Levels::WARN:
                   printf("\x1b[33m%s\x1b[0m\n", log_string);
                   break;
                 case Log_Levels::DEBUG:
                   printf("\x1b[32m%s\x1b[0m\n", log_string);
                   break;
                 case Log_Levels::INFO:
                   printf("%s\n", log_string);
                   break;
               }
             }));
  };

  ~ScopedPrintSink() { logger_->deregister_sink("print"); };

 private:
  std::shared_ptr<Logger> logger_;
};

#endif