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

	~Logger() = default;

	template <typename... Args>
	void debug(const char *f, Args &&... args) const {
		if (level <= Log_Levels::DEBUG) {
			auto now = std::chrono::system_clock::now().time_since_epoch();
			auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
			now -= seconds;
			auto nano_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now); 

			auto before_metadata = fmt::format(f, std::forward<Args>(args)...);
			auto after_metadata = fmt::format("DEBUG: [{}.{}] {}", seconds.count(), nano_seconds.count(), before_metadata);
			output(after_metadata);
		}
	}

	template <typename... Args>
	void info(const char *f, Args &&... args) const {
		if (level <= Log_Levels::INFO) {
			auto now = std::chrono::system_clock::now().time_since_epoch();
			auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
			now -= seconds;
			auto nano_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now); 

			auto before_metadata = fmt::format(f, std::forward<Args>(args)...);
			auto after_metadata = fmt::format("INFO: [{}.{}] {}", seconds.count(), nano_seconds.count(), before_metadata);
			output(after_metadata);
		}
	}

	template <typename... Args>
	void warn(const char *f, Args &&... args) const {
		if (level <= Log_Levels::WARN) {
			auto now = std::chrono::system_clock::now().time_since_epoch();
			auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
			now -= seconds;
			auto nano_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now); 

			auto before_metadata = fmt::format(f, std::forward<Args>(args)...);
			auto after_metadata = fmt::format("WARN: [{}.{}] {}", seconds.count(), nano_seconds.count(), before_metadata);
			output(after_metadata);
		}
	}

	template <typename... Args>
	void error(const char *f, Args &&... args) const {
		if (level <= Log_Levels::ERROR) {
			auto now = std::chrono::system_clock::now().time_since_epoch();
			auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
			now -= seconds;
			auto nano_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now); 

			auto before_metadata = fmt::format(f, std::forward<Args>(args)...);
			auto after_metadata = fmt::format("ERROR: [{}.{}] {}", seconds.count(), nano_seconds.count(), before_metadata);
			output(after_metadata);
		}
	}

	template <typename... Args>
	void fatal(const char *f, Args &&... args) const {
		if (level <= Log_Levels::FATAL) {
			auto now = std::chrono::system_clock::now().time_since_epoch();
			auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
			now -= seconds;
			auto nano_seconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now); 

			auto before_metadata = fmt::format(f, std::forward<Args>(args)...);
			auto after_metadata = fmt::format("FATAL: [{}.{}] {}", seconds.count(), nano_seconds.count(), before_metadata);
			output(after_metadata);
		}
	}


private:
	std::shared_ptr<rclcpp::node::Node> node;

	Log_Levels level;

	void output(const std::string &log_string) const {
		printf("%s\n", log_string.c_str());
	}
};