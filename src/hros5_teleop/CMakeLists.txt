cmake_minimum_required(VERSION 3.10)
project(hros5_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(head_teleop_node src/head_teleop_node.cpp)
ament_target_dependencies(head_teleop_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS head_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
