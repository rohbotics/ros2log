// This file represents helper functions that will be in rclcpp

#ifndef RCLCPP_LOGGING_H
#define RCLCPP_LOGGING_H

#include <ros2log/logger.hpp>
#include <ros2log/scoped_sinks.hpp>

namespace rclcpp {

void init_logger(std::shared_ptr<Logger> logger,
                 std::shared_ptr<rclcpp::node::Node> node) {
  logger->set_name(node->get_name());
  node->register_param_change_callback(
      [=](const std::vector<rclcpp::parameter::ParameterVariant>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (auto param : parameters) {
          for (auto& sink : logger->get_sinks()) {
            if ("logging/sinks/" + sink.name + "/level" == param.get_name()) {
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

                else {
                  result.successful = false;
                  result.reason =
                      "log_level must be one of: DEBUG, INFO, WARN, ERROR, "
                      "FATAL";
                }
              } else {
                result.successful = false;
                result.reason = "log_level must be a string";
              }
            }

            if ("logging/sinks/" + sink.name + "/enabled" == param.get_name()) {
              if (param.get_type() ==
                  rclcpp::parameter::ParameterType::PARAMETER_BOOL) {
                sink.enabled = param.get_value<bool>();

              } else {
                result.successful = false;
                result.reason = "log_level must be a boolean";
              }
            }
          }
        }

        return result;
      });

  static ScopedPrintSink print_sink(logger);
  static ScopedRosoutSink rosout_sink(logger, node);
}
}

#endif