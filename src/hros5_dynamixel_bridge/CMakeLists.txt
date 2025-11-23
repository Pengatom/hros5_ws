cmake_minimum_required(VERSION 3.8)
project(hros5_dynamixel_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)

add_executable(dynamixel_bridge_node src/dynamixel_bridge_node.cpp src/dynamixel_driver.cpp)
ament_target_dependencies(dynamixel_bridge_node rclcpp sensor_msgs trajectory_msgs yaml-cpp dynamixel_sdk)

add_executable(head_dxl_node src/head_dxl_node.cpp)
ament_target_dependencies(head_dxl_node
  rclcpp
  std_msgs
  dynamixel_sdk
)

add_executable(left_hand_dxl_node src/left_hand_dxl_node.cpp)
ament_target_dependencies(left_hand_dxl_node
  rclcpp
  std_msgs
  dynamixel_sdk
  yaml-cpp
  ament_index_cpp
)

target_include_directories(dynamixel_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dynamixel_bridge_node yaml-cpp)
target_link_libraries(left_hand_dxl_node yaml-cpp)

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

install(TARGETS dynamixel_bridge_node head_dxl_node left_hand_dxl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
