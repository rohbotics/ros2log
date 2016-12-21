#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h>

#include <chrono>


enum class Log_Levels
{
	DEBUG,
	INFO,
	WARN,
	ERROR,
	FATAL,
	NONE
};

class Logger
{
public:
	Logger(std::shared_ptr<rclcpp::node::Node> n) : node(n), level(Log_Levels::INFO) {

		std::vector<rclcpp::parameter::ParameterVariant> params;
		params.emplace_back("log_level", "INFO");
		node->set_parameters(params);

		node->register_param_change_callback(
			[=](const std::vector<rclcpp::parameter::ParameterVariant>& parameters) {
				rcl_interfaces::msg::SetParametersResult result;
				result.successful = true;

				for(auto param : parameters) {
					if ("log_level" == param.get_name()) {
						if (param.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
							std::string level_str = param.get_value<std::string>();

							if (level_str == "DEBUG")
								level = Log_Levels::DEBUG;

							else if (level_str == "INFO")
								level = Log_Levels::INFO;

							else if (level_str == "WARN")
								level = Log_Levels::WARN;

							else if (level_str == "ERROR")
								level = Log_Levels::ERROR;

							else if (level_str == "FATAL")
								level = Log_Levels::FATAL;

							else if (level_str == "NONE")
								level = Log_Levels::NONE;

							else {
								result.successful = false;
								result.reason = "log_level must be one of: DEBUG, INFO, WARN, ERROR, FATAL, NONE";
							}
						}
						else {
							result.successful = false;
							result.reason = "log_level must be a string";
						} 
					}
				}

				return result;
			}
		);


	};

	Logger() = default;
	~Logger() = default;

	void debug(const std::string& log_string) const {
		if (level <= Log_Levels::DEBUG) {
			output(log_string);
		}
	}

	void info(const std::string& log_string) const {
		if (level <= Log_Levels::INFO) {
			output(log_string);
		}
	}

	void warn(const std::string& log_string) const {
		if (level <= Log_Levels::WARN) {
			output(log_string);
		}
	}

	void error(const std::string& log_string) const {
		if (level <= Log_Levels::ERROR) {
			output(log_string);
		}
	}

	void fatal(const std::string& log_string) const {
		if (level <= Log_Levels::FATAL) {
			output(log_string);
		}
	}


private:
	std::shared_ptr<rclcpp::node::Node> node;

	Log_Levels level;

	void output(const std::string& log_string) const {
		printf("%s\n", log_string.c_str());
	}
};