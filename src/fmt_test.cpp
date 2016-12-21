#include <rclcpp/rclcpp.hpp>

#include <fmt/format.h>

#include <string>
#include <utility>

#include <roslog_fmt/logger.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = rclcpp::node::Node::make_shared("roslog_fmt_test");
	auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

	Logger log(node);


	rclcpp::WallRate loop_rate(1);

	int i = 0;
	while (rclcpp::ok()) {
		log.debug("Friendly debug {}", i);
		log.info("FYI {}", i);
		log.warn("Foo warning! {}", i);
		log.error("Bar error! {}", i);
		log.fatal("-----------------");
		i++;

		rclcpp::spin_some(node);
    	loop_rate.sleep();
	}
}
