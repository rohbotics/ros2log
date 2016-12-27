#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/log.hpp>
#include <string>

void topic_callback(const rosgraph_msgs::msg::Log::SharedPtr msg) {
  printf("%s\n", msg->msg.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("rosout_print");
  auto parameter_service =
      std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto subscription = node->create_subscription<rosgraph_msgs::msg::Log>(
      "rosout", topic_callback);
  rclcpp::spin(node);
}
