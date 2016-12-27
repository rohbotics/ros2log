cmake_minimum_required(VERSION 3.5)
project(ros2log)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)


include_directories(include ${rclcpp_INCLUDE_DIRS} ${fmt_INCLUDE_DIRS} ${rosgraph_msgs_INCLUDE_DIRS})
link_directories(${rclcpp_LIBRARY_DIRS})

add_executable(fmt_test src/fmt_test.cpp)
target_link_libraries(fmt_test
  fmt
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${rosgraph_msgs_LIBRARIES}
)

add_definitions(-std=c++11)

add_executable(rosout_print src/rosout_print.cpp)
target_link_libraries(rosout_print
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES} 
  ${rosgraph_msgs_LIBRARIES}
)

install(TARGETS
  fmt_test
  rosout_print
  DESTINATION bin)

ament_package()
