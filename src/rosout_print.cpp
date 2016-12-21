#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  printf("%s\n", msg->data.c_str());
}


int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = rclcpp::node::Node::make_shared("rosout_print");
	auto parameter_service = std::make_shared<rclcpp::parameter_service::ParameterService>(node);

	auto subscription = node->create_subscription<std_msgs::msg::String> ("rosout", topic_callback);
	rclcpp::spin(node);
}
