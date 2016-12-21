#include <rclcpp/rclcpp.hpp>

#include <fmt/format.h>

#include <string>
#include <utility>

#include <roslog_fmt/logger.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = rclcpp::node::Node::make_shared("roslog_fmt_test");
	Logger log(node);


	const int foo = 42;
	log.warn("The meaning of life {}", foo);
	log.warn("How about in hex: {:x}", foo);
	log.warn("Binary: {:b}", foo);

	double bar = 3.14159265;
	log.error("Pi {}", bar);

	std::string hi = "hola";
	log.error("{}", hi);

	rclcpp::WallRate loop_rate(1);

	int i = 0;
	while (rclcpp::ok()) {
		log.error("Hi! {}", i);
		log.warn("Hola! {}", i);
		i++;

		rclcpp::spin_some(node);
    	loop_rate.sleep();
	}
}
