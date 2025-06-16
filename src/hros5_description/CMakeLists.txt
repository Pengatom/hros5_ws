cmake_minimum_required(VERSION 3.8)
project(hros5_description)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(rviz2 REQUIRED)

install(
    DIRECTORY urdf launch rviz
    DESTINATION share/${PROJECT_NAME}
)
ament_package()