// This file represents helper functions that will be in rclcpp

#ifndef RCLCPP_LOGGING_H
#define RCLCPP_LOGGING_H

#include <ros2log/logger.hpp>

namespace rclcpp {

void init_logger(std::shared_ptr<Logger> logger, std::shared_ptr<rclcpp::node::Node> Node) {
  logger->register_sink(
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
}
}

#endif