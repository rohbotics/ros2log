#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h>


enum class Log_Levels
{
	DEBUG,
	INFO,
	WARN,
	ERROR
};

class Logger
{
public:
	Logger(std::shared_ptr<rclcpp::node::Node> n) : node(n), level(Log_Levels::ERROR) {
		node->register_param_change_callback(
			[=](const std::vector<rclcpp::parameter::ParameterVariant> parameters) -> rcl_interfaces::msg::SetParametersResult {
				rcl_interfaces::msg::SetParametersResult result;
				result.successful = true;

				for(auto param : parameters) {
					if ("log_level" == param.get_name()) {
						if (param.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_STRING){
							std::string level_str = param.get_value<std::string>();

							if (level_str == "ERROR")
								level = Log_Levels::ERROR;

							if (level_str == "WARN")
								level = Log_Levels::WARN;
						}
					}
				}

				return result;
			}
		);
	};

	~Logger() = default;

	template <typename... Args>
	void error(const char *f, Args &&... args) {
		if (level <= Log_Levels::ERROR){
			fmt::MemoryWriter w;
			w.write(f, std::forward<Args>(args)...);
			printf("%s\n", w.c_str());
		}
	}

	template <typename... Args>
	void warn(const char *f, Args &&... args) {
		if (level <= Log_Levels::WARN){
			fmt::MemoryWriter w;
			w.write(f, std::forward<Args>(args)...);
			printf("%s\n", w.c_str());
		}
	}


private:
	std::shared_ptr<rclcpp::node::Node> node;

	Log_Levels level;

};