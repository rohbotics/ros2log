// This file represents helper functions that will be in rclcpp

#ifndef RCLCPP_LOGGING_H
#define RCLCPP_LOGGING_H

#include <ros2log/logger.hpp>
#include <ros2log/scoped_sinks.hpp>

namespace rclcpp {

void init_logger(std::shared_ptr<Logger> logger, std::shared_ptr<rclcpp::node::Node> node) {
  static ScopedPrintSink print_sink(logger);
  static ScopedRosoutSink rosout_sink(logger, node);
}

}

#endif