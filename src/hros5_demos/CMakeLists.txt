cmake_minimum_required(VERSION 3.10)
project(hros5_demos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY
  	urdf
    launch
    config
    description
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.urdf"
                 PATTERN "*.xacro"
                 PATTERN "*.rviz"
                 PATTERN "*.yaml"
                 PATTERN "*.py"
)

ament_package()

