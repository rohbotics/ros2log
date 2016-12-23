#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <ros2log/enums.hpp>
#include <ros2log/sink.hpp>

#include <chrono>

class Logger {
 public:
  Logger(std::shared_ptr<rclcpp::node::Node> n)
      : node(n) {
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
                rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
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
                    "log_level must be one of: DEBUG, INFO, WARN, ERROR, FATAL, "
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

  virtual void register_sink(const Sink& sink) {
    sinks.push_back(sink);
  }

 protected:
  std::shared_ptr<rclcpp::node::Node> node;

  std::vector<Sink> sinks;

  virtual void output(Log_Levels level, const char * log_string) const {
    for (auto& sink : sinks) {
      if (level >= sink.output_level) {
        sink.output_function(level, log_string);
      }
    }
    // if (level >= print_level) {
    //   switch (color) {
    //     case Log_Color::RED:
    //       printf("\x1b[31m%s\x1b[0m\n", log_string);
    //       break;
    //     case Log_Color::YELLOW:
    //       printf("\x1b[33m%s\x1b[0m\n", log_string);
    //       break;
    //     case Log_Color::GREEN:
    //       printf("\x1b[32m%s\x1b[0m\n", log_string);
    //       break;
    //     case Log_Color::DEFAULT:
    //       printf("%s\n", log_string);
    //       break;
    //   }
    // }

    // if (level >= rosout_level) {
    //   auto message = std_msgs::msg::String();
    //   message.data = log_string;
    //   rosout_pub->publish(message);
    // }
  }
};