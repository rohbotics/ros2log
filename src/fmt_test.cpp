#include <rclcpp/rclcpp.hpp>

#include <fmt/format.h>

#include <string>
#include <utility>

#include <ros2log/fmt_logger.hpp>
#include <ros2log/sink.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("fmt_test");
  auto parameter_service =
      std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  FmtLogger log(node);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rosout_pub;
  rosout_pub = node->create_publisher<std_msgs::msg::String>("rosout");

  log.register_sink(Sink("print", Log_Levels::INFO, [=](Log_Levels level, const char * log_string) {
    switch (level) {
      case Log_Levels::FATAL:
      case Log_Levels::ERROR:
        printf("\x1b[31m%s\x1b[0m\n", log_string);
        break;
      case Log_Levels::WARN:
        printf("\x1b[33m%s\x1b[0m\n", log_string);
        break;
      case Log_Levels::DEBUG:
        printf("\x1b[32m%s\x1b[0m\n", log_string);
        break;
      case Log_Levels::INFO:
        printf("%s\n", log_string);
        break;
    }
  }));

  log.register_sink(Sink("rosout", Log_Levels::INFO, [=](Log_Levels level, const char * log_string) {
    auto message = std_msgs::msg::String();
    message.data = log_string;
    rosout_pub->publish(message);
  }));

  rclcpp::WallRate loop_rate(5);

  int i = 0;
  while (rclcpp::ok()) {
    log.debug("Friendly debug {}", i);
    log.info("FYI {}", i);
    log.warn("Foo warning! {}", i);
    log.error("Bar error! {}", i);
    log.fatal("----FATALITY----");
    i++;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}
