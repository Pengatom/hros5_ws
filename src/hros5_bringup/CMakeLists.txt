cmake_minimum_required(VERSION 3.5)
project(hros5_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs geometry_msgs tf2_ros nav_msgs std_msgs)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
