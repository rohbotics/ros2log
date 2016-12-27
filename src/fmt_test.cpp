#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/logging.hpp>

#include <fmt/format.h>

#include <string>
#include <utility>
  
#include <ros2log/fmt_logger.hpp>
#include <ros2log/sink.hpp>
#include <ros2log/scoped_sinks.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("fmt_test");
  auto parameter_service =
      std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  auto log = std::make_shared<FmtLogger>(node);

  rclcpp::init_logger(log, node);

  rclcpp::WallRate loop_rate(1);

  int i = 0;
  while (rclcpp::ok()) {
    LOG_DEBUG(log, "Friendly debug {}", i);
    LOG_INFO(log, "FYI {}", i);
    LOG_WARN(log, "Foo warning! {}", i);
    LOG_ERROR(log, "Bar error! {}", i);
    LOG_FATAL(log, "----FATALITY----");
    i++;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}
