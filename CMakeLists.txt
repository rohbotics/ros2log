cmake_minimum_required(VERSION 3.5)
project(roslog_fmt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${fmt_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})

add_executable(fmt_test src/fmt_test.cpp)
target_link_libraries(fmt_test
  fmt
  ${rclcpp_LIBRARIES} 
)

install(TARGETS
  fmt_test
  DESTINATION bin)

ament_package()
