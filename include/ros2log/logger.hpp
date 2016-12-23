#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

enum class Log_Levels { DEBUG, INFO, WARN, ERROR, FATAL, NONE };

enum class Log_Color { RED, YELLOW, GREEN, DEFAULT };

class Logger {
 public:
  Logger(std::shared_ptr<rclcpp::node::Node> n)
      : node(n), print_level(Log_Levels::INFO), rosout_level(Log_Levels::INFO) {
    std::vector<rclcpp::parameter::ParameterVariant> params;
    params.emplace_back("log_level", "INFO");
    node->set_parameters(params);

    node->register_param_change_callback([=](
        const std::vector<rclcpp::parameter::ParameterVariant>& parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (auto param : parameters) {
        if ("print_level" == param.get_name()) {
          if (param.get_type() ==
              rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            std::string level_str = param.get_value<std::string>();

            if (level_str == "DEBUG")
              print_level = Log_Levels::DEBUG;

            else if (level_str == "INFO")
              print_level = Log_Levels::INFO;

            else if (level_str == "WARN")
              print_level = Log_Levels::WARN;

            else if (level_str == "ERROR")
              print_level = Log_Levels::ERROR;

            else if (level_str == "FATAL")
              print_level = Log_Levels::FATAL;

            else if (level_str == "NONE")
              print_level = Log_Levels::NONE;

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
        if ("rosout_level" == param.get_name()) {
          if (param.get_type() ==
              rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
            std::string level_str = param.get_value<std::string>();

            if (level_str == "DEBUG")
              rosout_level = Log_Levels::DEBUG;

            else if (level_str == "INFO")
              rosout_level = Log_Levels::INFO;

            else if (level_str == "WARN")
              rosout_level = Log_Levels::WARN;

            else if (level_str == "ERROR")
              rosout_level = Log_Levels::ERROR;

            else if (level_str == "FATAL")
              rosout_level = Log_Levels::FATAL;

            else if (level_str == "NONE")
              rosout_level = Log_Levels::NONE;

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

      return result;
    });

    rosout_pub = node->create_publisher<std_msgs::msg::String>("rosout");
  };

  Logger() = default;
  ~Logger() = default;

 protected:
  std::shared_ptr<rclcpp::node::Node> node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rosout_pub;

  Log_Levels print_level;
  Log_Levels rosout_level;

  virtual void output(Log_Levels level, const std::string& log_string, Log_Color color) const {
    if (level >= print_level) {
      switch (color) {
        case Log_Color::RED:
          printf("\x1b[31m%s\x1b[0m\n", log_string.c_str());
          break;
        case Log_Color::YELLOW:
          printf("\x1b[33m%s\x1b[0m\n", log_string.c_str());
          break;
        case Log_Color::GREEN:
          printf("\x1b[32m%s\x1b[0m\n", log_string.c_str());
          break;
        case Log_Color::DEFAULT:
          printf("%s\n", log_string.c_str());
          break;
      }
    }

    if (level >= rosout_level) {
      auto message = std_msgs::msg::String();
      message.data = log_string;
      rosout_pub->publish(message);
    }
  }
};