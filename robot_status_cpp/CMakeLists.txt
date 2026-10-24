cmake_minimum_required(VERSION 3.8)
project(robot_status_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(robot_status_interfaces REQUIRED)

add_executable(status_publisher src/status_publisher.cpp)
target_link_libraries(status_publisher
  rclcpp::rclcpp
  ${robot_status_interfaces_TARGETS}
)

add_executable(drive_to_point_server src/drive_to_point_server.cpp)
target_link_libraries(drive_to_point_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${turtlesim_TARGETS}
  ${robot_status_interfaces_TARGETS}
)

install(TARGETS status_publisher drive_to_point_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
