#include <rclcpp/rclcpp.hpp>

#include <fmt/format.h>

#include <string>
#include <utility>

#include <roslog_fmt/fmt_logger.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("fmt_test");
  auto parameter_service =
      std::make_shared<rclcpp::parameter_service::ParameterService>(node);

  FmtLogger log(node);

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
