// This file represents helper functions that will be in rclcpp

#ifndef RCLCPP_LOGGING_H
#define RCLCPP_LOGGING_H

#include <ros2log/logger.hpp>
#include <ros2log/scoped_sinks.hpp>

namespace rclcpp {

static Logger* find_logger(const std::string& logger_name, Logger* logger) {
  auto slash = logger_name.find('/');
  auto top_logger = logger_name.substr(0, slash);
  for (auto child : logger->get_children()) {
    if (child->get_name().c_str() == top_logger) {
      if (slash == std::string::npos) {
        return child;  // There was no slash, so that was the end
      }
      return find_logger(logger_name.substr(slash + 1), child);
    } else
      return nullptr;
  }
  return nullptr;
}

void init_logger(std::shared_ptr<Logger> logger,
                 std::shared_ptr<rclcpp::node::Node> node) {
  logger->set_name(node->get_name());
  node->register_param_change_callback(
      [=](const std::vector<rclcpp::parameter::ParameterVariant>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
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
                      "level must be one of: DEBUG, INFO, WARN, ERROR, "
                      "FATAL";
                }
              } else {
                result.successful = false;
                result.reason = "level must be a string";
              }
            }

            if ("logging/sinks/" + sink.name + "/enabled" == param.get_name()) {
              if (param.get_type() ==
                  rclcpp::parameter::ParameterType::PARAMETER_BOOL) {
                sink.enabled = param.get_value<bool>();

              } else {
                result.successful = false;
                result.reason = "enabled must be a boolean";
              }
            }
          }

          // Special case for root logger
          if ("logging/level" == param.get_name()) {
            if (param.get_type() ==
                rclcpp::parameter::ParameterType::PARAMETER_STRING) {
              std::string level_str = param.get_value<std::string>();
              if (level_str == "DEBUG")
                logger->logger_level = Log_Levels::DEBUG;

              else if (level_str == "INFO")
                logger->logger_level = Log_Levels::INFO;

              else if (level_str == "WARN")
                logger->logger_level = Log_Levels::WARN;

              else if (level_str == "ERROR")
                logger->logger_level = Log_Levels::ERROR;

              else if (level_str == "FATAL")
                logger->logger_level = Log_Levels::FATAL;

              else {
                result.successful = false;
                result.reason =
                    "level must be one of: DEBUG, INFO, WARN, ERROR, "
                    "FATAL";
              }
            }
          }

          // Special case for root logger
          if ("logging/enabled" == param.get_name()) {
            if (param.get_type() ==
                rclcpp::parameter::ParameterType::PARAMETER_BOOL) {
              logger->enabled = param.get_value<bool>();

            } else {
              result.successful = false;
              result.reason = "enabled must be a boolean";
            }
          }

          // Deal with parameters for the subloggers
          if (param.get_name().find("logging/loggers/") != std::string::npos) {
            auto param_name = param.get_name();
            auto logger_name = param_name.substr(
                16, param_name.rfind('/') -
                        16);  // from logging/loggers/ (16 chars) to the last /
            auto child_logger = find_logger(logger_name, logger.get());
            if (child_logger != nullptr) {
              auto parameter = param_name.substr(param_name.rfind('/') + 1);
              if (parameter == "level") {
                if (param.get_type() ==
                    rclcpp::parameter::ParameterType::PARAMETER_STRING) {
                  std::string level_str = param.get_value<std::string>();
                  if (level_str == "DEBUG")
                    child_logger->logger_level = Log_Levels::DEBUG;

                  else if (level_str == "INFO")
                    child_logger->logger_level = Log_Levels::INFO;

                  else if (level_str == "WARN")
                    child_logger->logger_level = Log_Levels::WARN;

                  else if (level_str == "ERROR")
                    child_logger->logger_level = Log_Levels::ERROR;

                  else if (level_str == "FATAL")
                    child_logger->logger_level = Log_Levels::FATAL;

                  else {
                    result.successful = false;
                    result.reason =
                        "level must be one of: DEBUG, INFO, WARN, ERROR, "
                        "FATAL";
                  }
                }
              }
              if (parameter == "enabled") {
                if (param.get_type() ==
                    rclcpp::parameter::ParameterType::PARAMETER_BOOL) {
                  child_logger->enabled = param.get_value<bool>();

                } else {
                  result.successful = false;
                  result.reason = "enabled must be a boolean";
                }
              }
            } else {
              result.successful = false;
              result.reason = "Could not find logger" + logger_name;
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